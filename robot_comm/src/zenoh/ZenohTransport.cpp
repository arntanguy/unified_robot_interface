#include <robot_comm/zenoh/ZenohTransport.h>

#include <mc_rtc/logging.h>

#include <condition_variable>
#include <filesystem>
#include <thread>
#include <utility>

namespace fs = std::filesystem;

namespace robot_comm
{

zenoh::Config ZenohTransport::configureTransport(const mc_rtc::Configuration & config)
{
  auto zenoh_config = zenoh::Config::create_default();

  const std::string protocol = config("protocol");

  // TODO: consider give user warning instead of modifying zenoh_config
  if(protocol == "zenoh/shm")
  {
    zenoh_config.insert_json5("transport/shared_memory/enabled", "true");
    zenoh_config.insert_json5("mode", "\"peer\"");
  }

  if(config.has("configuration"))
  {
    const std::string path = config("configuration");

    if(!fs::exists(path))
    {
      mc_rtc::log::error_and_throw("[ZenohTransport] Configuration "
                                   "file does not exist: {}",
                                   path);
    }

    mc_rtc::log::info("[ZenohTransport] Loading "
                      "configuration: {}",
                      path);

    zenoh_config = zenoh::Config::from_file(path);
  }

  return zenoh_config;
}

ZenohTransport::ZenohTransport(const mc_rtc::Configuration & config) : TransportInterface(config)
{
  auto zenoh_config = configureTransport(config_);

  session_ = std::make_unique<zenoh::Session>(std::move(zenoh_config));
}

void ZenohTransport::start()
{
  mc_rtc::log::info("[ZenohTransport] Started");
}

void ZenohTransport::stop()
{
  liveliness_subscribers_.clear();
  liveliness_tokens_.clear();
  subscribers_.clear();
  publishers_.clear();
  queryables_.clear();

  mc_rtc::log::info("[ZenohTransport] Stopped");
}

bool ZenohTransport::publish(const std::string & topic, const ByteBuffer & payload)
{
  auto it = publishers_.find(topic);

  if(it == publishers_.end())
  {
    auto pub = session_->declare_publisher(topic);

    it = publishers_.emplace(topic, std::move(pub)).first;
  }

  it->second.put(payload);

  return true;
}

void ZenohTransport::subscribe(const std::string & topic, ReceiveCallback callback)
{
  if(hasSubscriber(topic))
  {
    return;
  }

  auto subscriber = session_->declare_subscriber(
      zenoh::KeyExpr(topic),
      [callback](const zenoh::Sample & sample)
      {
        auto bytes = sample.get_payload().as_vector();

        ByteBuffer payload(bytes.begin(), bytes.end());

        callback(static_cast<std::string>(sample.get_keyexpr().as_string_view()), payload);
      },
      zenoh::closures::none);

  subscribers_.emplace(topic, std::move(subscriber));

  mc_rtc::log::info("[ZenohTransport] Subscribed to '{}'", topic);
}

bool ZenohTransport::hasSubscriber(const std::string & topic) const
{
  return subscribers_.find(topic) != subscribers_.end();
}

void ZenohTransport::registerQueryable(const std::string & key_expr, QueryHandler handler)
{
  if(queryables_.count(key_expr))
  {
    mc_rtc::log::warning("[ZenohTransport] Queryable '{}' already registered, ignoring", key_expr);
    return;
  }

  auto queryable = session_->declare_queryable(
      zenoh::KeyExpr(key_expr),
      [handler](zenoh::Query & query)
      {
        ByteBuffer payload;
        if(auto p = query.get_payload())
        {
          auto bytes = p->get().as_vector();
          payload.assign(bytes.begin(), bytes.end());
        }
        ByteBuffer reply_payload = handler(payload);
        query.reply(query.get_keyexpr(), zenoh::Bytes(reply_payload));
      },
      zenoh::closures::none);

  queryables_.emplace(key_expr, std::move(queryable));

  mc_rtc::log::info("[ZenohTransport] Registered queryable '{}'", key_expr);
}

std::optional<ByteBuffer> ZenohTransport::query(const std::string & key_expr,
                                                const ByteBuffer & payload,
                                                std::chrono::milliseconds timeout)
{
  struct QueryState
  {
    std::mutex mutex;
    std::condition_variable cv;
    std::optional<ByteBuffer> result;
    bool done = false;
  };

  auto state = std::make_shared<QueryState>();

  auto options = zenoh::Session::GetOptions::create_default();
  options.payload = zenoh::Bytes(payload);
  options.timeout_ms = static_cast<uint64_t>(timeout.count());

  session_->get(
      zenoh::KeyExpr(key_expr), "",
      [state](zenoh::Reply & reply)
      {
        if(reply.is_ok())
        {
          auto bytes = reply.get_ok().get_payload().as_vector();
          std::lock_guard<std::mutex> lock(state->mutex);
          state->result = ByteBuffer(bytes.begin(), bytes.end());
        }
      },
      [state]()
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->done = true;
        state->cv.notify_one();
      },
      std::move(options));

  std::unique_lock<std::mutex> lock(state->mutex);
  // Wait slightly beyond the zenoh timeout so the on_drop fires first.
  state->cv.wait_for(lock, timeout + std::chrono::milliseconds(500), [&state] { return state->done; });

  if(!state->result)
  {
    mc_rtc::log::warning("[ZenohTransport] Query '{}' timed out or returned no reply", key_expr);
  }

  return state->result;
}

void ZenohTransport::declareLiveliness(const std::string & key_expr)
{
  if(liveliness_tokens_.count(key_expr)) return;

  liveliness_tokens_.emplace(key_expr, session_->liveliness_declare_token(zenoh::KeyExpr(key_expr)));

  mc_rtc::log::info("[ZenohTransport] Declared liveliness token '{}'", key_expr);
}

void ZenohTransport::watchLiveliness(const std::string & key_expr, LivelinessCallback callback)
{
  if(liveliness_subscribers_.count(key_expr))
  {
    mc_rtc::log::warning("[ZenohTransport] Liveliness of '{}' already watched, ignoring", key_expr);
    return;
  }

  // history: also report a token declared before this subscriber
  auto options = zenoh::Session::LivelinessSubscriberOptions::create_default();
  options.history = true;

  auto subscriber = session_->liveliness_declare_subscriber(
      zenoh::KeyExpr(key_expr), [callback](const zenoh::Sample & sample)
      { callback(sample.get_kind() == Z_SAMPLE_KIND_PUT); }, zenoh::closures::none, std::move(options));

  liveliness_subscribers_.emplace(key_expr, std::move(subscriber));

  mc_rtc::log::info("[ZenohTransport] Watching liveliness of '{}'", key_expr);
}

} // namespace robot_comm
