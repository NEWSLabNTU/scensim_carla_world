#!/usr/bin/env python3
"""Live check: the CARLA world server keeps the spec's physics on its cars across spawns.

CARLA 0.9.16 puts a blueprint's own tire friction (3.5 for the Model 3) back on live vehicles
of that blueprint whenever another vehicle of it spawns, by any client (scensim b1c0684; the car
whose physics was applied last; a sensor, a walker or another blueprint resets nothing). The world server re-applies the
spec after each spawn it makes and after each tick that shows it someone else's.

This drives a world server over the world protocol, as the bridge does: the ego, two agent
Model 3s, a scenario NPC Model 3 (stock: not the spec's), a Model 3 another client spawns,
and an Audi; after each spawn and step it reads every car's tires through a CARLA client
of its own. Every spec'd car must read the file's friction; the stock cars their own.

    python3 scripts/check_spec_tires.py --world tcp://127.0.0.1:5691 --carla-port 2000 \\
        --map-dir $CARLA_MAPS/Town01 \\
        --physics <share>/sim_vehicle_spec/generated/tesla_model3/carla_physics.yaml

Start the world server against an idle CARLA first (no bridge owns it), with that
`vehicle_physics`. Needs `carla` (0.9.16), `pyzmq`, `protobuf` and `pyyaml`; the protocol
module comes from deps/swb_world_protocol. Exit 0 when every read was right.
"""

import argparse
import math
import os
import socket
import sys
import time
from pathlib import Path

import carla
import yaml
import zmq

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "deps/swb_world_protocol/scensim_world_server"))
from scensim_world_server import protocol as pb  # noqa: E402
from scensim_world_server.lifetime import start_time  # noqa: E402

MODEL3 = "vehicle.tesla.model3"


class World:
    def __init__(self, endpoint):
        self.sock = zmq.Context.instance().socket(zmq.REQ)
        self.sock.setsockopt(zmq.RCVTIMEO, 300_000)
        self.sock.connect(endpoint)
        self.me = pb.ClientId(host=socket.gethostname(), pid=os.getpid(),
                              start_time=start_time(os.getpid()))

    def call(self, **request):
        self.sock.send(pb.Request(client=self.me, **request).SerializeToString())
        reply = pb.Reply.FromString(self.sock.recv())
        if not reply.ok:
            raise RuntimeError(f"{list(request)[0]} refused: {reply.error}")
        return reply


def world_pose(t):
    """A CARLA transform as the protocol's world frame (ROS: Y flipped, yaw negated)."""
    yaw = -math.radians(t.rotation.yaw)
    return pb.make_pose((t.location.x, -t.location.y, t.location.z + 0.3),
                        (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--world", default="tcp://127.0.0.1:5591")
    ap.add_argument("--carla-host", default="localhost")
    ap.add_argument("--carla-port", type=int, default=2000)
    ap.add_argument("--town", default="Town01")
    ap.add_argument("--map-dir", required=True, help="the town's converted map (CARLA_MAPS/<Town>)")
    ap.add_argument("--physics", required=True, help="the world server's vehicle_physics")
    ap.add_argument("--ego-role", default="hero")
    ap.add_argument("--hold", type=float, default=0.0,
                    help="s to step in real time after the ego spawns and after each spawn, so a "
                         "vehicle bridge (acb_bridge, carla_physics_mode verify) attaches and looks")
    a = ap.parse_args()

    spec = yaml.safe_load(Path(a.physics).read_text())
    want = spec["wheels"][0]["tire_friction"]
    ws = World(a.world)

    def load():
        return pb.LoadWorld(world=a.town, map_dir=a.map_dir, step_time=0.05, realtime_factor=1.0,
                            lanelet2_map_path=str(Path(a.map_dir) / "lanelet2_map.osm"))
    hello = ws.call(hello=pb.Hello(protocol_version=pb.PROTOCOL_VERSION)).hello
    print(f"world server: {hello.simulator_name} {hello.simulator_version}")
    ws.call(load_world=load())

    client = carla.Client(a.carla_host, a.carla_port)
    client.set_timeout(60)
    cw = client.get_world()
    points = cw.get_map().get_spawn_points()
    stock = {}  # actor id -> its own friction at spawn
    spec_ids, names, failures = set(), {}, []

    def read(tag):
        line = []
        for vid in list(names):
            v = cw.get_actor(vid)
            f = [round(w.tire_friction, 3) for w in v.get_physics_control().wheels]
            expect = want if v.id in spec_ids else stock[v.id]
            ok = all(abs(x - expect) < 1e-3 for x in f)
            line.append(f"{names[v.id]}={f[0]}{'' if ok else ' (WRONG: want %s)' % expect}")
            if not ok:
                failures.append(f"{tag}: {names[v.id]} {f}, want {expect}")
        print(f"  {tag:46s} {'  '.join(line)}", flush=True)

    def step(n=1):
        for _ in range(n):
            ws.call(step=pb.Step(substeps=1, dt=0.05))

    def hold():
        end = time.monotonic() + a.hold
        while time.monotonic() < end:
            step()
            time.sleep(0.05)

    def spawn(name, kind, motion, i, model=MODEL3, role=""):
        r = ws.call(spawn=pb.Spawn(name=name, kind=pb.KIND[kind], subtype="car",
                                   pose=world_pose(points[i]), model=model, asset_key=model,
                                   motion=pb.MOTION[motion], role_name=role))
        v = cw.get_actor(r.spawn.body.sim_id)
        names[v.id] = name
        stock.setdefault(v.id, 3.5 if model == MODEL3 else None)
        return v

    handles = []
    try:
        ego = spawn("ego", "EGO", "EXTERNAL", 1, role=a.ego_role)
        spec_ids.add(ego.id)
        step(3)
        read("ego spawned, 3 steps")
        hold()
        for k, i in (("coop_a", 10), ("coop_b", 20)):
            v = spawn(k, "VEHICLE", "EXTERNAL", i, role=k)
            spec_ids.add(v.id)
            read(f"agent {k} spawned (before a step)")
            step(2)
            read(f"agent {k}, 2 steps")
            hold()
        spawn("npc_m3", "VEHICLE", "KINEMATIC", 30)
        read("scenario NPC Model 3 spawned (before a step)")
        step(2)
        read("NPC, 2 steps")
        hold()
        bl = cw.get_blueprint_library()
        other = cw.spawn_actor(bl.find(MODEL3), points[40])
        names[other.id], stock[other.id] = "foreign_m3", 3.5
        handles.append(other)
        print(f"  (another client spawned Model 3 actor {other.id})")
        step(1)
        read("foreign Model 3, 1 step")
        step(2)
        read("foreign Model 3, 3 steps")
        hold()
        audi = cw.spawn_actor(bl.find("vehicle.audi.etron"), points[50])
        names[audi.id] = "foreign_etron"
        stock[audi.id] = audi.get_physics_control().wheels[0].tire_friction
        handles.append(audi)
        step(2)
        read("foreign Audi, 2 steps")
        imu = cw.spawn_actor(bl.find("sensor.other.imu"), carla.Transform(), attach_to=ego)
        handles.insert(0, imu)
        step(2)
        read("an IMU on the ego, 2 steps")
    finally:
        for h in handles:
            try:
                h.destroy()
            except RuntimeError:
                pass
        try:
            ws.call(load_world=load())
        except Exception as e:  # noqa: BLE001
            print(f"  (cleanup: {e})")
    if failures:
        print(f"FAIL: {len(failures)} wrong reads")
        for f in failures:
            print(f"  {f}")
        return 1
    print(f"PASS: every spec'd car on {want}, every stock car on its own")
    return 0


if __name__ == "__main__":
    sys.exit(main())
