#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <mc_rtc/Configuration.h>

#include <robot_comm/serialization/Buffer.h>

namespace robot_comm
{

/// Called with the topic and raw payload of each received message.
using ReceiveCallback = std::function<void(const std::string & topic, const ByteBuffer & payload)>;

/// Synchronous query handler: receives payload, returns reply payload.
using QueryHandler = std::function<ByteBuffer(const ByteBuffer & payload)>;

/// Called with true when a watched liveliness token appears, false when it is lost.
using LivelinessCallback = std::function<void(bool alive)>;

/// Moves raw bytes between processes. Implement it to add a transport
/// (see docs/Communication.md, "Extending").
class TransportInterface
{
public:
  TransportInterface(const mc_rtc::Configuration & config) : config_(config) {}

  virtual ~TransportInterface() = default;

  virtual void start() = 0;

  virtual void stop() = 0;

  virtual bool publish(const std::string & topic, const ByteBuffer & payload) = 0;

  virtual void subscribe(const std::string & topic, ReceiveCallback callback) = 0;

  virtual bool hasSubscriber(const std::string & topic) const = 0;

  /// Answer incoming queries on `key_expr` with `handler`.
  virtual void registerQueryable(const std::string & key_expr, QueryHandler handler) = 0;

  /// Send a blocking query and return the first reply, or nullopt on timeout.
  virtual std::optional<ByteBuffer> query(const std::string & key_expr,
                                          const ByteBuffer & payload,
                                          std::chrono::milliseconds timeout = std::chrono::seconds(5)) = 0;

  /// Announce that this endpoint is alive under `key_expr` until the transport stops.
  /// Transports without liveliness support ignore it.
  virtual void declareLiveliness(const std::string & /*key_expr*/) {}

  /// Call `callback` when a liveliness token matching `key_expr` appears or is lost.
  /// Transports without liveliness support never call it.
  virtual void watchLiveliness(const std::string & /*key_expr*/, LivelinessCallback /*callback*/) {}

protected:
  mc_rtc::Configuration config_;
};

} // namespace robot_comm
