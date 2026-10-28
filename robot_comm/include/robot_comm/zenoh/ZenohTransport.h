#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include <zenoh.hxx>

#include <robot_comm/transport/TransportInterface.h>

namespace robot_comm
{

class ZenohTransport : public TransportInterface
{
public:
  ZenohTransport(const mc_rtc::Configuration & config);

  ~ZenohTransport() override = default;

  void start() override;

  void stop() override;

  bool publish(const std::string & topic, const ByteBuffer & payload) override;

  void subscribe(const std::string & topic, ReceiveCallback callback) override;

  bool hasSubscriber(const std::string & topic) const override;

  void registerQueryable(const std::string & key_expr, QueryHandler handler) override;

  std::optional<ByteBuffer> query(const std::string & key_expr,
                                  const ByteBuffer & payload,
                                  std::chrono::milliseconds timeout = std::chrono::seconds(5)) override;

  void declareLiveliness(const std::string & key_expr) override;

  void watchLiveliness(const std::string & key_expr, LivelinessCallback callback) override;

private:
  zenoh::Config configureTransport(const mc_rtc::Configuration & config);

  std::unique_ptr<zenoh::Session> session_;

  std::unordered_map<std::string, zenoh::Publisher> publishers_;

  std::unordered_map<std::string, zenoh::Subscriber<void>> subscribers_;

  std::unordered_map<std::string, zenoh::Queryable<void>> queryables_;

  std::unordered_map<std::string, zenoh::LivelinessToken> liveliness_tokens_;

  std::unordered_map<std::string, zenoh::Subscriber<void>> liveliness_subscribers_;
};

} // namespace robot_comm
