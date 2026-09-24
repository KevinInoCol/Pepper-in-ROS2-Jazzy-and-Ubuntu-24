// Pepper Tutorial V9 (ROS 2 Jazzy). Licencia Apache 2.0.
//
// Conduce a Pepper con velocidades aleatorias, como el random_driver.cpp del V8 (basado en
// el tutorial "random_husky_driver"). Cada 0.1 s (10 Hz) publica en /pepper/cmd_vel un avance
// aleatorio entre 0 y 1 m/s y un giro aleatorio entre -1 y 1 rad/s. El plugin de la base
// recorta el avance al límite de Pepper (0.55 m/s) y suaviza los cambios con sus límites de
// aceleración y jerk.
//
// Ejecutar:  ros2 run random_pepper_driver random_driver

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class RandomDriver : public rclcpp::Node
{
public:
  RandomDriver()
  : Node("random_driver")
  {
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/pepper/cmd_vel", 100);
    timer_ = create_wall_timer(100ms, [this]() {publishRandom();});
    RCLCPP_INFO(get_logger(), "Publicando velocidades aleatorias en /pepper/cmd_vel");
  }

private:
  void publishRandom()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
    msg.angular.z = 2.0 * static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) - 1.0;
    pub_->publish(msg);
    RCLCPP_DEBUG(
      get_logger(), "linear.x=%.2f angular.z=%.2f", msg.linear.x, msg.angular.z);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RandomDriver>());
  rclcpp::shutdown();
  return 0;
}
