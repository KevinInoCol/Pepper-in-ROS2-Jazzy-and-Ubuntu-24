// Pepper Tutorial V9 (ROS 2 Jazzy + Gazebo Harmonic). Licencia Apache 2.0.
//
// Sonar de Pepper, equivalente a libgazebo_ros_range.so del V8. Gazebo Harmonic no tiene
// sensor de ultrasonido: el haz se simula con un lidar de 5x5 rayos (±0.52 rad) y este nodo
// publica la distancia más corta como sensor_msgs/Range, igual que el plugin del V8.
//
//   /pepper/sonar_front_scan (LaserScan) -> /pepper/sonar_front (Range)
//   /pepper/sonar_back_scan  (LaserScan) -> /pepper/sonar_back  (Range)

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>

class SonarToRange : public rclcpp::Node
{
public:
  SonarToRange()
  : Node("sonar_to_range")
  {
    // Parámetros del V8 (<fov>, <minRange>, <maxRange>)
    fov_ = declare_parameter("fov", 1.05);
    min_range_ = declare_parameter("min_range", 0.0);
    max_range_ = declare_parameter("max_range", 5.0);
    const auto sonars = declare_parameter<std::vector<std::string>>(
      "sonars", {"sonar_front", "sonar_back"});

    for (const auto & name : sonars) {
      auto pub = create_publisher<sensor_msgs::msg::Range>("/pepper/" + name, 10);
      subs_.push_back(
        create_subscription<sensor_msgs::msg::LaserScan>(
          "/pepper/" + name + "_scan", rclcpp::SensorDataQoS(),
          [this, pub](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
            pub->publish(toRange(*scan));
          }));
      pubs_.push_back(pub);
    }
  }

private:
  sensor_msgs::msg::Range toRange(const sensor_msgs::msg::LaserScan & scan) const
  {
    sensor_msgs::msg::Range range;
    range.header = scan.header;
    range.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    range.field_of_view = fov_;
    range.min_range = min_range_;
    range.max_range = max_range_;
    // Sin eco: max_range, como gazebo_ros_range
    range.range = max_range_;
    for (const float r : scan.ranges) {
      if (std::isfinite(r) && r >= scan.range_min && r < range.range) {
        range.range = r;
      }
    }
    return range;
  }

  double fov_;
  double min_range_;
  double max_range_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subs_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> pubs_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonarToRange>());
  rclcpp::shutdown();
  return 0;
}
