// Pepper Tutorial V9 (ROS 2 Jazzy + Gazebo Harmonic). Licencia Apache 2.0.
//
// Publica /clock a 100 Hz, como gazebo_ros en ROS 1 (parámetro pub_clock_frequency = 100).
// Gazebo Harmonic publica el reloj en cada paso de física (1 kHz con pasos de 1 ms) y cada
// nodo con use_sim_time procesa todos esos mensajes: con Nav2 son ~40 suscriptores y la
// máquina se satura. La física sigue a 1 kHz; sólo se reduce la frecuencia del /clock de ROS.
//
//   /pepper/clock_raw (1 kHz, desde ros_gz_bridge) -> /clock (100 Hz)

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class ClockThrottle : public rclcpp::Node
{
public:
  ClockThrottle()
  : Node("clock_throttle")
  {
    const double frequency = declare_parameter("pub_clock_frequency", 100.0);
    period_ns_ = static_cast<int64_t>(1e9 / frequency);
    pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/pepper/clock_raw", rclcpp::ClockQoS(),
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
        const int64_t t = rclcpp::Time(msg->clock).nanoseconds();
        if (t < last_ns_ || t - last_ns_ >= period_ns_) {  // t < last: la simulación se reinició
          pub_->publish(*msg);
          last_ns_ = t;
        }
      });
  }

private:
  int64_t period_ns_;
  int64_t last_ns_{-1000000000LL};
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockThrottle>());
  rclcpp::shutdown();
  return 0;
}
