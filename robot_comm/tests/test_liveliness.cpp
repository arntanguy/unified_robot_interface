#include "config.h"
#include <robot_comm/CommunicationFactory.h>

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <functional>
#include <thread>

namespace fs = std::filesystem;

using namespace robot_comm;
using namespace std::chrono_literals;

namespace
{

mc_rtc::Configuration networkConfig()
{
  mc_rtc::Configuration config;
  config.add("protocol", std::string{"zenoh"});
  config.add("configuration", (fs::path(TEST_CONFIG_DIR) / "zenoh/client.json5").string());
  return config;
}

// Poll `condition` until it holds or `timeout` expires.
bool waitFor(const std::function<bool()> & condition, std::chrono::milliseconds timeout = 2s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while(std::chrono::steady_clock::now() < deadline)
  {
    if(condition()) return true;
    std::this_thread::sleep_for(10ms);
  }
  return condition();
}

} // namespace

TEST(LivelinessTest, PeerNeverSeenCountsAsAlive)
{
  auto server = CommunicationFactory::makeCommunication("liveliness_alone", networkConfig());
  server->setupServer();

  std::this_thread::sleep_for(300ms);

  EXPECT_TRUE(server->peerAlive());
}

TEST(LivelinessTest, ServerDetectsLostClient)
{
  auto server = CommunicationFactory::makeCommunication("liveliness_client_lost", networkConfig());
  auto client = CommunicationFactory::makeCommunication("liveliness_client_lost", networkConfig());
  server->setupServer();
  client->setupClient();

  std::this_thread::sleep_for(300ms);
  ASSERT_TRUE(server->peerAlive());

  client.reset();

  EXPECT_TRUE(waitFor([&] { return !server->peerAlive(); })) << "server did not notice the client going away";
}

TEST(LivelinessTest, ClientDetectsLostServerAndItsReturn)
{
  auto server = CommunicationFactory::makeCommunication("liveliness_server_lost", networkConfig());
  auto client = CommunicationFactory::makeCommunication("liveliness_server_lost", networkConfig());
  server->setupServer();
  client->setupClient();

  std::this_thread::sleep_for(300ms);
  ASSERT_TRUE(client->peerAlive());

  server.reset();
  EXPECT_TRUE(waitFor([&] { return !client->peerAlive(); })) << "client did not notice the server going away";

  server = CommunicationFactory::makeCommunication("liveliness_server_lost", networkConfig());
  server->setupServer();
  EXPECT_TRUE(waitFor([&] { return client->peerAlive(); })) << "client did not notice the server coming back";
}

TEST(LivelinessTest, RxAgeTracksLastMessage)
{
  auto server = CommunicationFactory::makeCommunication("liveliness_rx_age", networkConfig());
  auto client = CommunicationFactory::makeCommunication("liveliness_rx_age", networkConfig());
  server->setupServer();
  client->setupClient();

  EXPECT_FALSE(server->rxAge().has_value());

  std::this_thread::sleep_for(300ms);
  ASSERT_TRUE(waitFor(
      [&]
      {
        client->send(ByteBuffer{1, 2, 3});
        return server->rxAge().has_value();
      }));

  // Messages still in flight from the loop above may land a bit later.
  std::this_thread::sleep_for(150ms);
  EXPECT_GE(*server->rxAge(), 100ms);

  client->send(ByteBuffer{4, 5, 6});
  EXPECT_TRUE(waitFor([&] { return *server->rxAge() < 100ms; }));
}
