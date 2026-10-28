#include <robot_comm/CommunicationZenoh.h>
#include <robot_interface/FMInterfaceTemplate.h>

#include <mc_control/mc_global_controller.h>

#include <fmt/core.h>

namespace robot_manager
{

namespace
{
ControlMode parseModeString(const std::string & mode)
{
  if(mode == "velocity") return VELOCITY;
  if(mode == "torque") return TORQUE;
  return POSITION;
}

// Time the robot gets to send its first state once the control loop runs
constexpr auto first_state_timeout = std::chrono::seconds(10);
} // namespace

FMInterfaceTemplate::FMInterfaceTemplate(const std::string & name,
                                         const mc_rtc::Configuration & config,
                                         uint8_t buffer_size)
: RobotInterfaceBase(name, config, (buffer_size == 0) ? 6 : buffer_size)
{
  fmt::print("[FMInterfaceTemplate] '{}' starting\n", name);

  const std::string mode = config("controller")("mode", std::string{"position"});
  control_mode_ = parseModeString(mode);
  fmt::print("[FMInterfaceTemplate] '{}' control mode: {}\n", name, mode);

  connection_timeout_ = std::chrono::milliseconds(config("network_interface")("connection_timeout_ms", 100));

  mc_rtc::Configuration com_config(config("network_interface"));
  auto comm = robot_comm::CommunicationFactory::makeCommunication(name, com_config);
  comm->setupServer();
  setCommunication(std::move(comm));

  fmt::print("[FMInterfaceTemplate] '{}' ready\n", name);
}

void FMInterfaceTemplate::checkConnection()
{
  if(!connection_error_.empty()) return;

  const auto now = std::chrono::steady_clock::now();
  if(!first_update_) first_update_ = now;

  if(!communication().peerAlive())
  {
    connection_error_ = "robot_interface process gone";
    return;
  }

  if(!gc_initialized_)
  {
    if(now - *first_update_ > first_state_timeout)
      connection_error_ = fmt::format("no state received within {} s", first_state_timeout.count());
    return;
  }

  const auto age = communication().rxAge();
  if(age && *age > connection_timeout_)
  {
    connection_error_ =
        fmt::format("no state for {} ms", std::chrono::duration_cast<std::chrono::milliseconds>(*age).count());
  }
}

void FMInterfaceTemplate::updateSensors(robot_controller::Controller & gc)
{
  checkConnection();

  std::chrono::steady_clock::time_point arrival;
  auto rx = communication().receive(&arrival);
  if(!rx || rx->empty()) return;

  auto s = communication().serializer()->deserialize<robot_comm::State>(robot_comm::MessageType::STATE, rx->data(),
                                                                        rx->size());
  if(!s) return;

  if(s->stamp != 0 && s->stamp != last_state_stamp_)
  {
    last_state_stamp_ = s->stamp;
    last_state_arrival_ = arrival;
  }

  if(!s->position.empty()) gc.setEncoderValues(name(), s->position);
  if(!s->velocity.empty()) gc.setEncoderVelocities(name(), s->velocity);
  if(!s->torque.empty()) gc.setJointTorques(name(), s->torque);

  for(const auto & bs : s->bodySensors)
    gc.setBodySensor(name(), bs.name, bs.orientation, bs.angularVelocity, bs.linearAcceleration);

  for(const auto & fs : s->forceSensors) gc.setForceSensor(name(), fs.name, fs.force, fs.torque);

  if(!gc_initialized_ && !s->position.empty())
  {
    gc.initializeRobot(name(), s->position);

    gc_initialized_ = true;

    fmt::print("[FMInterfaceTemplate] '{}' robot initialized\n", name());
  }
}

void FMInterfaceTemplate::updateControl(robot_controller::Controller & gc)
{
  if(!gc_initialized_) return;

  robot_comm::Command command;

  switch(control_mode_)
  {
    case POSITION:
      command.position = gc.command(name(), robot_controller::ControlMode::POSITION);
      break;

    case VELOCITY:
      command.velocity = gc.command(name(), robot_controller::ControlMode::VELOCITY);
      break;

    case TORQUE:
      command.torque = gc.command(name(), robot_controller::ControlMode::TORQUE);
      break;
  }

  if(last_state_stamp_ != 0)
  {
    command.stateStamp = last_state_stamp_;
    command.stateHold = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - last_state_arrival_)
            .count());
  }

  if(!communication().send(communication().encode(command)))
    fmt::print("[FMInterfaceTemplate][warning] '{}' failed to send command\n", name());
}

} // namespace robot_manager
