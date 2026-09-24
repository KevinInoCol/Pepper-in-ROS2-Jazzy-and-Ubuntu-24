// Copyright 2018 Sammy Pfeiffer, The Magic Lab, University of Technology Sydney.
// Licencia Apache 2.0.
//
// Port a ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8) para el Pepper Tutorial V9.
//
// Controlador de modelo que aplica un Twist (/pepper/cmd_vel) como velocidad del robot
// (base holonómica: x, y, yaw), con límites de velocidad, aceleración y jerk, y publica
// odometría con ruido gaussiano integrada a partir de la velocidad medida (acumula deriva,
// como un robot real) más la TF odom -> base_footprint.
//
// Mismos parámetros SDF que el plugin del V8 (<robotNamespace>, <commandTopic>,
// <outputVelocityTopic>, <updateRate>, <commandTimeout>, <odometryTopic>, <odometryFrame>,
// <odometryRate>, <publishOdometryTf>, <robotBaseFrame>, <gaussianNoiseXY>,
// <gaussianNoiseYaw>, <linear|angular><Velocity|Acceleration|Jerk>Limit>).
//
// Diferencias con el V8:
//  - La velocidad lateral tenía el signo invertido (vy_mundo = x·sin(yaw) − y·cos(yaw)):
//    con linear.y > 0 el robot iba a su derecha. Aquí +y es la izquierda (convención ROS).
//  - La odometría se integra con el punto medio del giro. La fórmula de arco del V8 sumaba
//    vx + vy como si fuera una distancia en línea recta, lo que es incorrecto para una base
//    holonómica que se desplaza en lateral mientras gira.
//  - El tiempo es el de la simulación (UpdateInfo::simTime).

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "gazebo_model_velocity_plugin/speed_limiter.hpp"

namespace gazebo_model_velocity_plugin
{

namespace
{
double toSeconds(const std::chrono::steady_clock::duration & d)
{
  return std::chrono::duration<double>(d).count();
}

rclcpp::Time toRosTime(const std::chrono::steady_clock::duration & d)
{
  return rclcpp::Time(
    std::chrono::duration_cast<std::chrono::nanoseconds>(d).count(), RCL_ROS_TIME);
}

template<typename T>
T param(const std::shared_ptr<const sdf::Element> & sdf, const std::string & name, T def)
{
  return sdf->Get<T>(name, def).first;
}
}  // namespace

class GazeboRosModelVelocity
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  ~GazeboRosModelVelocity() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm)) {
      gzerr << "GazeboRosModelVelocity debe ir dentro de un <model>." << std::endl;
      return;
    }
    link_ = gz::sim::Link(model_.CanonicalLink(ecm));
    link_.EnableVelocityChecks(ecm, true);

    const auto ns = param<std::string>(sdf, "robotNamespace", "");
    const auto command_topic = param<std::string>(sdf, "commandTopic", "cmd_vel");
    const auto output_vel_topic = param<std::string>(sdf, "outputVelocityTopic", "output_vel");
    const auto odometry_topic = param<std::string>(sdf, "odometryTopic", "odom");
    update_rate_ = param<double>(sdf, "updateRate", 20.0);
    command_timeout_ = param<double>(sdf, "commandTimeout", 0.5);
    odometry_frame_ = param<std::string>(sdf, "odometryFrame", "odom");
    odometry_rate_ = param<double>(sdf, "odometryRate", 20.0);
    publish_odometry_tf_ = param<bool>(sdf, "publishOdometryTf", true);
    robot_base_frame_ = param<std::string>(sdf, "robotBaseFrame", "base_footprint");
    gaussian_noise_xy_ = param<double>(sdf, "gaussianNoiseXY", 0.0);
    gaussian_noise_yaw_ = param<double>(sdf, "gaussianNoiseYaw", 0.0);

    const double lin_vel = param<double>(sdf, "linearVelocityLimit", 100.0);
    const double ang_vel = param<double>(sdf, "angularVelocityLimit", 10.0);
    const double lin_acc = param<double>(sdf, "linearAccelerationLimit", 10.0);
    const double ang_acc = param<double>(sdf, "angularAccelerationLimit", 10.0);
    const double lin_jerk = param<double>(sdf, "linearJerkLimit", 100.0);
    const double ang_jerk = param<double>(sdf, "angularJerkLimit", 1000.0);
    limiter_lin_ = SpeedLimiter(
      true, true, true, -lin_vel, lin_vel, -lin_acc, lin_acc, -lin_jerk, lin_jerk);
    limiter_ang_ = SpeedLimiter(
      true, true, true, -ang_vel, ang_vel, -ang_acc, ang_acc, -ang_jerk, ang_jerk);

    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = rclcpp::Node::make_shared("gazebo_ros_model_velocity", ns);
    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      command_topic, 1, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_cmd_ = *msg;
        last_command_time_ = sim_time_;
        received_command_ = true;
      });
    output_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(output_vel_topic, 1);
    odometry_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odometry_topic, 1);
    if (publish_odometry_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    }

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    RCLCPP_INFO(
      node_->get_logger(), "GazeboRosModelVelocity: escuchando %s, odometría en %s",
      cmd_sub_->get_topic_name(), odometry_pub_->get_topic_name());
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || !node_) {
      return;
    }
    const double now = toSeconds(info.simTime);

    std::lock_guard<std::mutex> lock(mutex_);
    sim_time_ = now;

    const double dt = now - last_velocity_update_time_;
    if (dt >= 1.0 / update_rate_) {
      geometry_msgs::msg::Twist cmd = current_cmd_;
      if (now - last_command_time_ > command_timeout_) {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
      }
      limiter_lin_.limit(cmd.linear.x, last_cmd0_.linear.x, last_cmd1_.linear.x, dt);
      limiter_lin_.limit(cmd.linear.y, last_cmd0_.linear.y, last_cmd1_.linear.y, dt);
      limiter_ang_.limit(cmd.angular.z, last_cmd0_.angular.z, last_cmd1_.angular.z, dt);
      last_cmd1_ = last_cmd0_;
      last_cmd0_ = cmd;
      output_vel_pub_->publish(cmd);
      last_velocity_update_time_ = now;
    }

    // La física de Gazebo Harmonic descarta el comando de velocidad del modelo tras cada
    // paso: hay que aplicarlo en todos los pasos (el limitador corre a <updateRate>).
    // Hasta el primer comando el robot queda libre para asentarse en el suelo.
    if (received_command_) {
      applyVelocity(ecm, last_cmd0_);
    }

    if (odometry_rate_ > 0.0) {
      const double since_odom = now - last_odom_publish_time_;
      if (since_odom >= 1.0 / odometry_rate_) {
        publishOdometry(ecm, since_odom, info.simTime);
        last_odom_publish_time_ = now;
      }
    }
  }

private:
  void applyVelocity(gz::sim::EntityComponentManager & ecm, const geometry_msgs::msg::Twist & cmd)
  {
    // Gazebo interpreta LinearVelocityCmd/AngularVelocityCmd de un modelo en el marco del
    // modelo. Se fija la velocidad horizontal y el giro en yaw y se conserva la velocidad
    // vertical actual, para que la gravedad siga actuando.
    const gz::math::Pose3d pose = gz::sim::worldPose(model_.Entity(), ecm);
    const double yaw = pose.Rot().Yaw();
    const double vx_world = cmd.linear.x * std::cos(yaw) - cmd.linear.y * std::sin(yaw);
    const double vy_world = cmd.linear.x * std::sin(yaw) + cmd.linear.y * std::cos(yaw);
    double vz_world = 0.0;
    if (const auto v = link_.WorldLinearVelocity(ecm)) {
      vz_world = v->Z();
    }
    const gz::math::Vector3d linear =
      pose.Rot().RotateVectorReverse({vx_world, vy_world, vz_world});
    const gz::math::Vector3d angular =
      pose.Rot().RotateVectorReverse({0.0, 0.0, cmd.angular.z});

    auto * lin_cmd = ecm.Component<gz::sim::components::LinearVelocityCmd>(model_.Entity());
    if (lin_cmd) {
      *lin_cmd = gz::sim::components::LinearVelocityCmd(linear);
    } else {
      ecm.CreateComponent(model_.Entity(), gz::sim::components::LinearVelocityCmd(linear));
    }
    auto * ang_cmd = ecm.Component<gz::sim::components::AngularVelocityCmd>(model_.Entity());
    if (ang_cmd) {
      *ang_cmd = gz::sim::components::AngularVelocityCmd(angular);
    } else {
      ecm.CreateComponent(model_.Entity(), gz::sim::components::AngularVelocityCmd(angular));
    }
  }

  double gaussian(double sigma)
  {
    if (sigma <= 0.0) {
      return 0.0;
    }
    return std::normal_distribution<double>(0.0, sigma)(rng_);
  }

  void publishOdometry(
    const gz::sim::EntityComponentManager & ecm, double step_time,
    const std::chrono::steady_clock::duration & sim_time)
  {
    const auto pose = link_.WorldPose(ecm);
    const auto lin = link_.WorldLinearVelocity(ecm);
    const auto ang = link_.WorldAngularVelocity(ecm);
    if (!pose || !lin || !ang) {
      return;
    }
    // Velocidad en el marco del robot, con ruido (como la odometría de las ruedas)
    const gz::math::Vector3d body_lin = pose->Rot().RotateVectorReverse(*lin);
    const gz::math::Vector3d body_ang = pose->Rot().RotateVectorReverse(*ang);
    double vx = body_lin.X() + gaussian(gaussian_noise_xy_);
    double vy = body_lin.Y() + gaussian(gaussian_noise_xy_);
    double wz = body_ang.Z() + gaussian(gaussian_noise_yaw_);
    // Robot parado: sin ruido (igual que el V8)
    if (last_cmd0_.linear.x == 0.0) {vx = 0.0;}
    if (last_cmd0_.linear.y == 0.0) {vy = 0.0;}
    if (last_cmd0_.angular.z == 0.0) {wz = 0.0;}

    const double dtheta = wz * step_time;
    const double mid_yaw = odom_yaw_ + 0.5 * dtheta;
    odom_x_ += (vx * std::cos(mid_yaw) - vy * std::sin(mid_yaw)) * step_time;
    odom_y_ += (vx * std::sin(mid_yaw) + vy * std::cos(mid_yaw)) * step_time;
    odom_yaw_ += dtheta;

    const rclcpp::Time stamp = toRosTime(sim_time);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odometry_frame_;
    odom.child_frame_id = robot_base_frame_;
    odom.pose.pose.position.x = odom_x_;
    odom.pose.pose.position.y = odom_y_;
    odom.pose.pose.orientation.z = std::sin(0.5 * odom_yaw_);
    odom.pose.pose.orientation.w = std::cos(0.5 * odom_yaw_);
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wz;

    // Covarianzas del V8
    const bool turning = std::abs(body_ang.Z()) >= 0.0001;
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[14] = 1e12;
    odom.pose.covariance[21] = 1e12;
    odom.pose.covariance[28] = 1e12;
    odom.pose.covariance[35] = turning ? 100.0 : 0.01;
    odom.twist.covariance[0] = 0.001;
    odom.twist.covariance[7] = 0.001;
    odom.twist.covariance[14] = 0.001;
    odom.twist.covariance[21] = 1e12;
    odom.twist.covariance[28] = 1e12;
    odom.twist.covariance[35] = turning ? 100.0 : 0.01;
    odometry_pub_->publish(odom);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = robot_base_frame_;
      tf.transform.translation.x = odom_x_;
      tf.transform.translation.y = odom_y_;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex mutex_;
  geometry_msgs::msg::Twist current_cmd_;
  geometry_msgs::msg::Twist last_cmd0_;
  geometry_msgs::msg::Twist last_cmd1_;
  bool received_command_{false};
  double sim_time_{0.0};
  double last_command_time_{0.0};
  double last_velocity_update_time_{0.0};
  double last_odom_publish_time_{0.0};

  double update_rate_{20.0};
  double command_timeout_{0.5};
  double odometry_rate_{20.0};
  bool publish_odometry_tf_{true};
  std::string odometry_frame_;
  std::string robot_base_frame_;
  double gaussian_noise_xy_{0.0};
  double gaussian_noise_yaw_{0.0};
  SpeedLimiter limiter_lin_;
  SpeedLimiter limiter_ang_;

  std::mt19937 rng_{0};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
};

}  // namespace gazebo_model_velocity_plugin

GZ_ADD_PLUGIN(
  gazebo_model_velocity_plugin::GazeboRosModelVelocity,
  gz::sim::System,
  gazebo_model_velocity_plugin::GazeboRosModelVelocity::ISystemConfigure,
  gazebo_model_velocity_plugin::GazeboRosModelVelocity::ISystemPreUpdate)
