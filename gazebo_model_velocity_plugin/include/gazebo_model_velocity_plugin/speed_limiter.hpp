// Copyright 2018 Sammy Pfeiffer (gazebo_model_velocity_plugin, V8), basado en
// ros_control/diff_drive_controller speed_limiter. Licencia Apache 2.0.
// Port a ROS 2 / Gazebo Harmonic para el Pepper Tutorial V9: sin cambios de lógica.

#ifndef GAZEBO_MODEL_VELOCITY_PLUGIN__SPEED_LIMITER_HPP_
#define GAZEBO_MODEL_VELOCITY_PLUGIN__SPEED_LIMITER_HPP_

namespace gazebo_model_velocity_plugin
{

class SpeedLimiter
{
public:
  SpeedLimiter(
    bool has_velocity_limits = false, bool has_acceleration_limits = false,
    bool has_jerk_limits = false, double min_velocity = 0.0, double max_velocity = 0.0,
    double min_acceleration = 0.0, double max_acceleration = 0.0,
    double min_jerk = 0.0, double max_jerk = 0.0);

  /// Limita v según velocidad, aceleración y jerk. v0 y v1 son los dos comandos anteriores.
  /// Devuelve el factor de limitación (1.0 si no se limitó).
  double limit(double & v, double v0, double v1, double dt);
  double limit_velocity(double & v);
  double limit_acceleration(double & v, double v0, double dt);
  double limit_jerk(double & v, double v0, double v1, double dt);

  bool has_velocity_limits;
  bool has_acceleration_limits;
  bool has_jerk_limits;
  double min_velocity;
  double max_velocity;
  double min_acceleration;
  double max_acceleration;
  double min_jerk;
  double max_jerk;
};

}  // namespace gazebo_model_velocity_plugin

#endif  // GAZEBO_MODEL_VELOCITY_PLUGIN__SPEED_LIMITER_HPP_
