# Communication (`robot_comm`)

`robot_comm` is the only link between the robot manager (`uri`) and each
robot-side `uri interface`. Neither side depends on how the bytes travel: they
exchange plain C++ structs (`State`, `Command`, a YAML `std::string`) through a
`Communication` object. That object pairs a **transport** with a
**serializer**.

```text
            robot_comm::Communication
        ┌────────────────────────────────────┐
State ─►│ ISerializer ──► TransportInterface │──► network / shared memory
Command │ (FlatBuffers,   (Zenoh, ...)       │
 YAML   │  Protobuf)                         │
        └────────────────────────────────────┘
```

## Topics

Every robot has a name, which is its key under `Robots:` in `mc_rtc.yaml`. All
of its traffic uses topics prefixed with that name:

| Topic | Pattern | Direction | Payload |
|---|---|---|---|
| `{name}/init` | query / reply | `uri manager` → `uri interface` | the robot's full config as a YAML string; reply `"OK"` or `"ERROR: <reason>"` |
| `{name}/state` | pub / sub | `uri interface` → `uri manager` | `robot_comm::State` |
| `{name}/command` | pub / sub | `uri manager` → `uri interface` | `robot_comm::Command` |
| `uri/alive/{name}/manager`, `uri/alive/{name}/interface` | Zenoh liveliness token | each side watches the other's | none; the token disappears when its process or session is gone |

The two sides take fixed roles. The manager side calls `setupServer()`: it
sends on `/command` and receives on `/state`. The robot side calls
`setupClient()`, which is the mirror image. After that, `send()` and
`receive()` use the right topics automatically.

`receive()` does not block and returns only the **latest** message. Older
messages are overwritten, never queued. This fits real-time control, where a
stale state or command is useless.

`setupServer()` and `setupClient()` also declare a liveliness token for their
side and watch the other side's. `peerAlive()` turns false once the peer was
seen and then lost (crash, `kill -9`, network drop), and true again when it
comes back. `rxAge()` gives the time since the last received message. Both
sides use them to detect a lost connection, see
[Robot manager](docRobotManager.html#connection-loss).

## Messages

Defined in `robot_comm/serialization/Messages.h`:

```cpp
struct State {                      // robot -> manager, once per robot cycle
  std::vector<double> position;     // [rad], in the RobotModule's ref joint order
  std::vector<double> velocity;     // [rad/s], may be empty
  std::vector<double> torque;       // [Nm]
  std::vector<BodySensorData> bodySensors;   // IMUs, by RobotModule sensor name
  std::vector<ForceSensorData> forceSensors; // F/T sensors, in sensor frame
  uint64_t stamp;                   // sender steady clock [ns] at send time
};

struct Command {                    // manager -> robot
  double kp, kd;                    // gains (reserved)
  std::vector<double> position;     // exactly one of these three is filled,
  std::vector<double> velocity;     // according to the robot's
  std::vector<double> torque;       // controller.mode
  uint64_t stateStamp;              // State::stamp of the latest state received
  uint64_t stateHold;               // [ns] that state was held before sending
};
```

The init payload is a `std::string` tagged `MessageType::CONFIG`. The mapping
from C++ type to `MessageType` is in `MessageTraits.h`.

The wire schemas are in
`robot_comm/include/robot_comm/serialization/flatbuffers/mc_rtc_msgs.fbs` and
`.../protobuf/mc_rtc_msgs.proto`. If you add a field to `State` or `Command`,
update both schemas **and** the conversion code in
`FlatBufferSerializer.cpp` / `ProtobufSerializer.cpp`.

## Configuration

Communication is configured per robot, under `network_interface`:

```yaml
Robots:
  ur5e:
    network_interface:
      protocol: zenoh          # required: zenoh | zenoh/shm
      backend: flatbuffer      # optional: flatbuffer (default) | protobuf
      configuration: /path/to/zenoh.json5   # optional: full Zenoh config file
      latency_stats: true      # optional: log robot_comm round trip (default false)
      connection_timeout_ms: 100   # optional: connection watchdog (default 100)
```

| Key | Values | Notes |
|---|---|---|
| `protocol` | `zenoh`, `zenoh/shm` | `zenoh/shm` enables Zenoh's shared-memory transport in peer mode. The manager picks it by default for `autostart` robots. `tcp`, `udp` and `shm` are parsed but not implemented yet, and are rejected by `CommunicationFactory`. |
| `backend` | `flatbuffer`, `protobuf` | Checked, but `ZenohCommunication` always uses FlatBuffers for now. Protobuf also requires building with Protobuf (`WITH_PROTOBUF`). |
| `configuration` | path to a `.json5` | Replaces the generated Zenoh config entirely, including the `zenoh/shm` settings. Use it to set `connect.endpoints` when multicast discovery is not available. |
| `latency_stats` | `true`, `false` | `uri interface` logs the robot_comm round trip about once per second. Read from the interface's own config: with `autostart` that is the manager's entry, otherwise the robot-side file. |
| `connection_timeout_ms` | milliseconds | Time without a new state (manager) or command (interface) after which the connection counts as lost. Read by each side from its own config, like `latency_stats`. Keep it well above `controller.time_step`. |

### Measuring latency

Every `State` carries `stamp`, the interface's steady clock at send time. The
manager echoes the latest one in each `Command` as `stateStamp`, with
`stateHold`, the time between that state's arrival and the command being sent.
When the command arrives, the interface computes:

```text
round trip = arrival - stateStamp - stateHold
```

Both stamps come from the interface's clock, so this works across machines
without clock sync. It covers serialization, the transport and delivery to the
receive buffer in both directions, but not the time mc_rtc spends between
commands. `one-way ~ round trip / 2` assumes both directions are symmetric.
Arrival times are taken in the transport callback, so the control loops'
polling delay is not counted.

```text
[info] [RobotInterface] 'ur5e' robot_comm round trip (<n> samples): min <x> us | avg <y> us | max <z> us (one-way ~ <y/2> us)
```

Example Zenoh client config that connects to a known router:

```json5
{
  mode: "client",
  connect: { endpoints: ["tcp/192.168.1.10:7447"] },
  scouting: { multicast: { enabled: false } },
}
```

## Using `robot_comm` directly

You normally never touch `robot_comm`: `uri manager` and `uri interface` already use
it. It is still a standalone library, handy for tools, tests or a custom
manager.

### Role-based state/command exchange

This is what `FMInterfaceTemplate` (manager side) and `RobotInterface` (robot
side) do every cycle:

```cpp
#include <robot_comm/CommunicationFactory.h>

mc_rtc::Configuration net;
net.add("protocol", "zenoh");

// Robot side: publishes on "demo/state", listens on "demo/command".
auto robot = robot_comm::CommunicationFactory::makeCommunication("demo", net);
robot->setupClient();

// Manager side: publishes on "demo/command", listens on "demo/state".
auto manager = robot_comm::CommunicationFactory::makeCommunication("demo", net);
manager->setupServer();

robot_comm::State state;
state.position = {0.0, -1.57, 1.57, 0.0, 0.0, 0.0};
robot->send(robot->encode(state));

// ... later, non-blocking:
if(auto rx = manager->receive(); rx && !rx->empty())
{
  auto s = manager->serializer()->deserialize<robot_comm::State>(
      robot_comm::MessageType::STATE, rx->data(), rx->size());
  // s->position == {0.0, -1.57, ...}
}
```

### Query / reply

This is the pattern behind the init handshake. The replying side registers a
handler; the other side blocks until it gets a reply or times out:

```cpp
// Robot side: answer queries on "demo/init".
robot->handleQuery("demo/init", [&](const robot_comm::ByteBuffer & payload) {
  auto yaml = robot->serializer()->deserialize<std::string>(
      robot_comm::MessageType::CONFIG, payload.data(), payload.size());
  const std::string ok = yaml ? "OK" : "ERROR: bad payload";
  return robot_comm::ByteBuffer(ok.begin(), ok.end());
});

// Manager side: send the config and wait up to 5 s.
auto payload = manager->serializer()->serialize(std::string{"controller: {mode: position}"});
auto reply = manager->query("demo/init", payload, std::chrono::seconds(5));
if(!reply) { /* timed out: is the robot process running? */ }
```

Zenoh needs a moment to propagate a newly declared queryable. Query too soon
after `handleQuery()` and the query can time out. The manager handles this by
retrying up to 5 times.

### Generic typed pub/sub

Any type with a `MessageTraits` specialization can be published on an
arbitrary topic:

```cpp
auto sub = manager->subscribe<robot_comm::State>("demo/state",
    [](const robot_comm::State & s) { /* called from the transport thread */ });
robot->publish("demo/state", state);
```

## Extending

- **A new transport**: implement `robot_comm::TransportInterface`
  (`publish`, `subscribe`, `registerQueryable`, `query`, ...). Add a
  `Communication` subclass that creates it, as `ZenohCommunication` does, and
  map a new `protocol` string to it in `CommunicationFactory::makeCommunication`.
- **A new serializer**: implement `robot_comm::ISerializer`
  (`serializeImpl`/`deserializeImpl` for each `MessageType`), then add a
  `SerializationBackend` value and a case in the `Communication` constructor.

The integration tests in `robot_interface/tests/` (`test_init_handshake.cpp`,
`test_pubsub_loop.cpp`) exercise exactly the patterns above. Build them with
`-DBUILD_TESTING=ON`.

---

[![CNRS-AIST JRL (Joint Robotics Laboratory)](images/jrl-logo.png)](https://unit.aist.go.jp/isri/isri-jrl/en/)[![CNRS](images/cnrs-logo.png)](https://www.cnrs.fr/)[![AIST](images/aist-logo.png)](https://www.aist.go.jp/index_en.html)

Copyright © CNRS-AIST JRL 2026
