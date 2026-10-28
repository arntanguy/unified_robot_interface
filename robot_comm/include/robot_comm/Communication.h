#pragma once

#include <robot_comm/Subscriber.h>
#include <robot_comm/serialization/Serializer.h>
#include <robot_comm/transport/TransportInterface.h>

#include <mc_rtc/Configuration.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot_comm
{

/// Messaging endpoint of one robot: typed pub/sub, query/reply, and a
/// role-based state/command channel. Create it with CommunicationFactory.
/// See docs/Communication.md.
class Communication
{
public:
  /// \param name Robot name, used as the prefix of the role-based topics.
  /// \param config The `network_interface` configuration.
  /// \param backend Serializer used for typed messages.
  Communication(std::string name,
                const mc_rtc::Configuration & config,
                SerializationBackend backend = SerializationBackend::Flatbuffer);

  virtual ~Communication();

  /// Start the transport.
  void start();
  /// Stop the transport.
  void stop();

  // -------------------------------------------------------
  // Typed pub/sub
  // -------------------------------------------------------

  /// Serialize and publish `msg` on `topic`.
  template<typename T>
  bool publish(const std::string & topic, const T & msg);

  /// Call `cb(const T &)` for every message received on `topic`. Keep the
  /// returned subscriber alive for as long as you need the callback.
  template<typename T, typename Callback>
  std::shared_ptr<Subscriber<T>> subscribe(const std::string & topic, Callback && cb);

  template<typename T, typename Handler>
  void registerQueryable(const std::string & topic, Handler && handler);

  template<typename T>
  std::optional<T> query(const std::string & topic, std::chrono::milliseconds timeout = std::chrono::seconds(1));

  void dispatch(const std::string & topic, const ByteBuffer & payload);

  // -------------------------------------------------------
  // Query / reply
  // -------------------------------------------------------

  /// Answer queries on `topic` with `handler`.
  void handleQuery(const std::string & topic, QueryHandler handler);

  /// Send a blocking query. Returns the reply, or nullopt if there is none
  /// within `timeout`.
  std::optional<ByteBuffer> query(const std::string & topic,
                                  const ByteBuffer & payload,
                                  std::chrono::milliseconds timeout = std::chrono::seconds(5));

  // -------------------------------------------------------
  // Convenience send / receive (role-based)
  // Uses fixed topics: {name}/state and {name}/command.
  // Call setupServer() or setupClient() once before use.
  // -------------------------------------------------------

  /// Manager role: send on {name}/command, receive on {name}/state.
  void setupServer();
  /// Robot role: send on {name}/state, receive on {name}/command.
  void setupClient();

  /// Serialize `msg` for send().
  template<typename T>
  ByteBuffer encode(const T & msg);

  /// Publish `buffer` on the role's send topic.
  bool send(const ByteBuffer & buffer);

  /// Latest buffer received on the role's receive topic, without blocking.
  /// Returns an empty buffer if nothing new arrived, nullopt on error. If
  /// `arrival` is set, it receives the arrival time (steady clock).
  std::optional<ByteBuffer> receive(std::chrono::steady_clock::time_point * arrival = nullptr);

  /// False once the peer endpoint (interface for the server, manager for the
  /// client) was seen and then lost, as reported by the transport's
  /// liveliness. True again when it comes back.
  bool peerAlive() const
  {
    return !peer_lost_;
  }

  // -------------------------------------------------------
  // Accessors
  // -------------------------------------------------------

  /// Serializer used by this endpoint.
  inline std::shared_ptr<ISerializer> serializer() const
  {
    return serializer_;
  }

protected:
  mc_rtc::Configuration parseConfig(const mc_rtc::Configuration & config);

  SerializationBackend backendFromString(const std::string & backend);

  Protocol protocolFromString(const std::string & protocol);

protected:
  std::string name_;

  std::shared_ptr<ISerializer> serializer_;

  std::shared_ptr<TransportInterface> transport_;

  std::unordered_map<std::string, std::vector<std::shared_ptr<SubscriberBase>>> subscribers_;

  mutable std::mutex mutex_;

  std::atomic<bool> running_ = false;

private:
  /// Declare this endpoint's liveliness token and watch the peer's, under
  /// uri/alive/{name}/{role}.
  void setupLiveliness(const std::string & role, const std::string & peer_role);

  // Role-based send/receive state
  std::string tx_topic_;
  std::string rx_topic_;
  std::optional<ByteBuffer> latest_rx_;
  std::chrono::steady_clock::time_point latest_rx_time_;
  std::mutex rx_mutex_;
  std::shared_ptr<SubscriberBase> rx_subscriber_;

  // Peer liveliness
  std::atomic<bool> peer_seen_ = false;
  std::atomic<bool> peer_lost_ = false;
};

//------------------------------------------------------------
// Typed publish
//------------------------------------------------------------

template<typename T>
bool Communication::publish(const std::string & topic, const T & msg)
{
  auto payload = serializer_->serialize(msg);

  return transport_->publish(topic, payload);
}

//------------------------------------------------------------
// Subscriber creation
//------------------------------------------------------------

template<typename T, typename Callback>
std::shared_ptr<Subscriber<T>> Communication::subscribe(const std::string & topic, Callback && cb)
{
  auto subscriber = std::make_shared<Subscriber<T>>(topic, serializer_, std::forward<Callback>(cb));

  bool create_transport_subscription = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    auto & subscribers = subscribers_[topic];

    create_transport_subscription = subscribers.empty();

    subscribers.push_back(subscriber);
  }

  if(create_transport_subscription)
  {
    transport_->subscribe(topic,
                          [this](const std::string & topic, const ByteBuffer & payload) { dispatch(topic, payload); });
  }

  return subscriber;
}

//------------------------------------------------------------
// Encode convenience
//------------------------------------------------------------

template<typename T>
ByteBuffer Communication::encode(const T & msg)
{
  return serializer_->serialize(msg);
}

} // namespace robot_comm
