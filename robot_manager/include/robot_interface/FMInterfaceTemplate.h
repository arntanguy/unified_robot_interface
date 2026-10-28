#pragma once

#include <robot_interface/RobotInterfaceBase.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace robot_manager
{

/// Default manager-side proxy: joint state and sensors in, one command type
/// (`controller.mode`) out.
class FMInterfaceTemplate : public RobotInterfaceBase
{
public:
  FMInterfaceTemplate(const std::string & name, const mc_rtc::Configuration & config, uint8_t buffer_size = 6);

  void init() override {}
  void reset() override {}
  void stop() override {}
  void updateSensors(robot_controller::Controller & gc) override;
  void updateControl(robot_controller::Controller & gc) override;

  [[nodiscard]] bool isInitialized() const override
  {
    return gc_initialized_;
  }

  [[nodiscard]] std::string connectionError() const override
  {
    return connection_error_;
  }

private:
  void checkConnection();

  bool gc_initialized_ = false;

  // Connection watchdog (network_interface.connection_timeout_ms)
  std::chrono::milliseconds connection_timeout_;
  std::optional<std::chrono::steady_clock::time_point> first_update_;
  std::string connection_error_;

  // Echoed in each Command for latency measurement.
  uint64_t last_state_stamp_ = 0;
  std::chrono::steady_clock::time_point last_state_arrival_;
};

} // namespace robot_manager
