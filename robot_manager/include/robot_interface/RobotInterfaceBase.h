#pragma once

#include <robot_comm/Communication.h>
#include <robot_comm/CommunicationFactory.h>
#include <robot_controller/Controller.h>

#include <mc_rtc/Configuration.h>

#include <condition_variable>

namespace robot_manager
{

enum ControlMode
{
  POSITION = 0,
  VELOCITY,
  TORQUE
};

/// Manager-side proxy of one robot: turns its State messages into controller
/// inputs and the controller output into Command messages. Subclass it for
/// robots that need custom behavior (see docs/RobotManager.md).
class RobotInterfaceBase
{
public:
  RobotInterfaceBase() = default;

  /// \param name Robot name (its key under `Robots:`).
  /// \param config The robot's configuration entry.
  /// \param buffer_size Size of the state buffer.
  RobotInterfaceBase(std::string name, mc_rtc::Configuration config, uint8_t buffer_size)
  : name_(std::move(name)), config_(std::move(config)), dt_(config_("controller")("time_step")),
    buffer_size_(buffer_size) {};

  virtual ~RobotInterfaceBase() = default;
  RobotInterfaceBase(const RobotInterfaceBase &) = delete;
  RobotInterfaceBase & operator=(const RobotInterfaceBase &) = delete;
  RobotInterfaceBase(RobotInterfaceBase &&) = delete;
  RobotInterfaceBase & operator=(RobotInterfaceBase &&) = delete;

  virtual void init() = 0;
  virtual void reset() = 0;
  virtual void stop() = 0;

  /// Pass the latest robot state to the controller.
  virtual void updateSensors(robot_controller::Controller & gc) = 0;
  /// Send the controller's command to the robot.
  virtual void updateControl(robot_controller::Controller & gc) = 0;

  /// True once the robot's first state has been passed to the controller.
  /// The controller only runs once every robot is initialized.
  [[nodiscard]] virtual bool isInitialized() const
  {
    return true;
  }

  /// Why the connection to the robot counts as lost, or empty while it is
  /// fine. The manager stops the controller as soon as one robot reports an
  /// error. Updated by updateSensors().
  [[nodiscard]] virtual std::string connectionError() const
  {
    return {};
  }

  /**
   * @brief Method in charge of robot sensors and commands update
   *
   */
  void controlThread(robot_controller::Controller & controller,
                     std::mutex & startM,
                     std::condition_variable & start_cv_,
                     bool & start) {
    // Not include `bool & running` like usual.
    // The value can be extract or change using methods defined in robot_controller::Controller
    // - isRunning()
    // - start()
    // - stop()
  };

  template<typename cm>
  void control();

  // TODO: use loadConfig instead of constructor to process
  void loadConfig(const mc_rtc::Configuration & config);

  /// Set the communication used to talk to the robot.
  void setCommunication(std::unique_ptr<robot_comm::Communication> communication)
  {
    communication_ = std::move(communication);
  }

  /// Robot name.
  [[nodiscard]] const std::string & name() const
  {
    return name_;
  }
  /// The robot's configuration entry.
  [[nodiscard]] const mc_rtc::Configuration & config() const
  {
    return config_;
  }
  /// Robot control period [s] (`controller.time_step`).
  [[nodiscard]] double dt() const
  {
    return dt_;
  }
  [[nodiscard]] uint8_t bufferSize() const
  {
    return buffer_size_;
  }

  /// Communication used to talk to the robot.
  [[nodiscard]] robot_comm::Communication & communication()
  {
    return *communication_;
  }

private:
  mutable std::mutex update_sensor_mtx_{};
  mutable std::mutex update_control_mtx_{};

  const std::string name_{};
  const mc_rtc::Configuration config_{};
  const double dt_{};
  const uint8_t buffer_size_{};

  std::vector<double> state_{};
  std::vector<double> command_{};

  std::unique_ptr<robot_comm::Communication> communication_{};

protected:
  ControlMode control_mode_ = POSITION;
};

} // namespace robot_manager
