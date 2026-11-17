# scensim_carla_world

[CARLA](https://carla.org/) 0.9.16 behind the world protocol of
[scenario-world-bridge](https://github.com/NEWSLabNTU/scenario-world-bridge): the CARLA
**world server**. It loads the town, spawns and moves the scenario's vehicles and people
(SSv2's NPCs teleported or tracked with physics, Traffic Manager autopilots, Autoware-driven
`agent` vehicles), sets traffic lights, weather and road friction, ticks CARLA synchronously
and reports contacts. It is the only code of the project that uses carla-rust.

One server per CARLA: it holds `/tmp/scensim-locks/carla-<host>-<port>.lock` for its whole
life, and exits 75 for its supervisor when CARLA goes away.

```bash
git submodule update --init --recursive   # deps/swb_world_protocol
just build
play_launch launch scensim_carla_world scensim_carla_world.launch.xml port:=5591 carla_port:=2000
scensim_world_check conformance tcp://127.0.0.1:5591 --map <map dir>   # from scensim_world_protocol
```

| Path | What |
|---|---|
| `scensim_carla_world/` | The ROS package (ament_cargo): the server, `launch/scensim_carla_world.launch.xml`, `config/scensim_carla_world.yaml`, the measured blueprint table `config/blueprint_boxes_0.9.16.yaml` |
| `scripts/measure_blueprint_boxes.py` | Re-measures that table (boxes and axles) from a running CARLA |
| `deps/swb_world_protocol` | The protocol and its Rust shell (`scensim_core`), a submodule |
| `deps/sim_vehicle_spec` | The vehicle specs; the ego's `carla_physics.yaml` comes from here, a submodule |

The CARLA version is the `carla-0916` feature of the `carla` dependency (`Cargo.toml`); a
`CARLA_DIR` in the environment breaks the build (carla-sys reads it as a source tree).

## The ego's physics

A scenario's ego can run the car its vehicle spec describes (roadmap 051, K3). The server's
ROS parameter `vehicle_physics` names a `carla_physics.yaml` (for the Tesla, the installed
`sim_vehicle_spec/generated/tesla_model3/carla_physics.yaml`); the default, `none`, leaves
the blueprint's own physics, so runs made before the specs drive the same cars. Given a
file, the server applies it before the spawn's settling tick, reads it back after that tick
and refuses an actor that differs. The spawn reply names the spec's `spec_hash`, which the
bridge compares with the vehicle side's.

The server then keeps it on. CARLA 0.9.16 puts a blueprint's own tire friction (3.5 for the
Model 3) back on live cars of that blueprint whenever another one spawns, by any client (the
rest of the physics stays; a sensor, a walker or another blueprint resets nothing). So after
each spawn of its own, before the settling tick, and after each tick whose world snapshot
shows a new actor id (someone else's spawn), the server reads every spec'd car back and
re-applies the file -- times the road's friction factor -- to any that lost it, and logs
`'<name>' lost its vehicle spec's physics when ... spawned`. Its own spawns cost the cars
nothing; another client's costs one tick. `scripts/check_spec_tires.py` checks it on a live
CARLA (the ego, two agent Model 3s, a scenario NPC, another client's Model 3 and Audi).

## Time and episodes

Simulation time is `nanos(elapsed_seconds) + BASE_NS` (`scensim_core::sim_time`); the episode is
CARLA's world id, in `LoadWorldReply.episode_id` and `StepReply.episode_id`. A world load
starts a new episode, and every vehicle stack of the old one ends (acb_bridge exits).

## Weather

`SetWeather` takes a CARLA preset by name (`ClearNoon`, `CloudySunset`, `HardRainNight`, ...;
any case, `_`/`-`/spaces ignored) or CARLA's `WeatherParameters` fields by name; fields not
named keep their current values. `GetWeather` returns all fourteen and the preset they
match. Through the bridge:

```bash
ros2 service call /scenario_world_bridge/set_weather scensim_interfaces/srv/SetWeather "{preset: ClearNoon}"
ros2 service call /scenario_world_bridge/set_weather scensim_interfaces/srv/SetWeather \
    "{names: [cloudiness, sun_altitude_angle], values: [30.0, 60.0]}"
```

| field | range |
|---|---|
| `cloudiness`, `precipitation` (rain), `precipitation_deposits` (puddles), `wind_intensity`, `fog_density`, `wetness` | 0-100 |
| `fog_distance` | metres where fog starts |
| `sun_azimuth_angle` | degrees, 0-360 |
| `sun_altitude_angle` | degrees, -90 (midnight) to 90 (midday) |
| `fog_falloff`, `scattering_intensity`, `mie_scattering_scale`, `rayleigh_scattering_scale`, `dust_storm` | as CARLA's [`WeatherParameters`](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.WeatherParameters) |

The server keeps the weather asked for -- `weather:` in `scensim_carla_world.yaml` (checked at
start), then the last `SetWeather` -- and applies it again after every world load and
reconnect.

## Spectator

`spectator: follow | once | off` and `spectator_height` (m, default 50) in
`scensim_carla_world.yaml` move CARLA's spectator straight down over the ego -- with no ego, the
first vehicle spawned, then the first entity -- after every step (`follow`) or once per
target (`once`); `off`, the default, leaves it alone. A camera someone moved (more than 0.1 m
or 1 deg from where the server put it) stays theirs until the next `LoadWorld`.

## Map props

Some towns place movable props in the level itself: Town01 has plastic chairs (0.5 kg),
tables (5 kg) and cones (24 kg) by the road, CARLA type `static.prop.mesh`. No scenario
declares them, so SSv2 cannot see them, and an NPC that brushes one can knock it into the
ego's lane. `map_props: frozen | live` in `scensim_carla_world.yaml` decides what happens to
them. `frozen`, the default, turns physics off on every `static.prop.mesh` the server did not
spawn, after each world load (every `LoadWorld`, so a world CARLA reloaded or restarted with
is covered too), and logs the count:

    Map props: froze 115 of 115 static.prop.mesh actor(s) in Town01 (map_props: frozen)

They keep their colliders and stay where the level put them. `live` leaves them to CARLA, as
before 2026-11-16. It is a world-server setting, not a host one, because it changes results;
`scensim run` records it in each run's provenance (`world_server.map_props`). Props a scenario
spawns are never touched, and a contact with a prop still counts.
