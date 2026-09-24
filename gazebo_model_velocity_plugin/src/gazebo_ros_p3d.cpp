// Pepper Tutorial V9 (ROS 2 Jazzy + Gazebo Harmonic). Licencia Apache 2.0.
//
// Posición real ("ground truth") de un link, equivalente a libgazebo_ros_p3d.so de Gazebo
// Classic, que el V8 usa para /pepper/odom_groundtruth. Mismos parámetros SDF:
// <bodyName>, <topicName>, <frameName> (sólo "world"), <updateRate>, <robotNamespace>.
//
// Publica nav_msgs/Odometry con la pose del link en el mundo y su velocidad lineal y angular
// en el marco del mundo, como gazebo_ros_p3d.

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo_model_velocity_plugin
{

class GazeboRosP3D
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  ~GazeboRosP3D() override
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
    const gz::sim::Model model(entity);
    body_name_ = sdf->Get<std::string>("bodyName", "base_footprint").first;
    const auto topic = sdf->Get<std::string>("topicName", "odom_groundtruth").first;
    frame_name_ = sdf->Get<std::string>("frameName", "world").first;
    const double rate = sdf->Get<double>("updateRate", 20.0).first;
    const auto ns = sdf->Get<std::string>("robotNamespace", "").first;
    period_ = rate > 0.0 ? 1.0 / rate : 0.0;

    if (frame_name_ != "world") {
      gzwarn << "GazeboRosP3D: sólo se admite <frameName>world</frameName>." << std::endl;
    }
    // Al pasar de URDF a SDF, los links unidos por joints fijos se fusionan con su padre:
    // si el link pedido no existe, se usa el link canónico (base_footprint en Pepper).
    auto link_entity = model.LinkByName(ecm, body_name_);
    if (link_entity == gz::sim::kNullEntity) {
      link_entity = model.CanonicalLink(ecm);
    }
    link_ = gz::sim::Link(link_entity);
    link_.EnableVelocityChecks(ecm, true);

    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = rclcpp::Node::make_shared("gazebo_ros_p3d", ns);
    pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(topic, 1);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() {executor_->spin();});
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || !node_) {
      return;
    }
    const double now = std::chrono::duration<double>(info.simTime).count();
    if (now - last_publish_time_ < period_) {
      return;
    }
    const auto pose = link_.WorldPose(ecm);
    const auto lin = link_.WorldLinearVelocity(ecm);
    const auto ang = link_.WorldAngularVelocity(ecm);
    if (!pose || !lin || !ang) {
      return;
    }
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = rclcpp::Time(
      std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count(), RCL_ROS_TIME);
    msg.header.frame_id = frame_name_;
    msg.child_frame_id = body_name_;
    msg.pose.pose.position.x = pose->Pos().X();
    msg.pose.pose.position.y = pose->Pos().Y();
    msg.pose.pose.position.z = pose->Pos().Z();
    msg.pose.pose.orientation.x = pose->Rot().X();
    msg.pose.pose.orientation.y = pose->Rot().Y();
    msg.pose.pose.orientation.z = pose->Rot().Z();
    msg.pose.pose.orientation.w = pose->Rot().W();
    msg.twist.twist.linear.x = lin->X();
    msg.twist.twist.linear.y = lin->Y();
    msg.twist.twist.linear.z = lin->Z();
    msg.twist.twist.angular.x = ang->X();
    msg.twist.twist.angular.y = ang->Y();
    msg.twist.twist.angular.z = ang->Z();
    pub_->publish(msg);
    last_publish_time_ = now;
  }

private:
  gz::sim::Link link_{gz::sim::kNullEntity};
  std::string body_name_;
  std::string frame_name_;
  double period_{0.05};
  double last_publish_time_{-1.0};

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

}  // namespace gazebo_model_velocity_plugin

GZ_ADD_PLUGIN(
  gazebo_model_velocity_plugin::GazeboRosP3D,
  gz::sim::System,
  gazebo_model_velocity_plugin::GazeboRosP3D::ISystemConfigure,
  gazebo_model_velocity_plugin::GazeboRosP3D::ISystemPostUpdate)
