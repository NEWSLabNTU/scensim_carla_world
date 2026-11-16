//! The spec's physics on a CARLA vehicle (roadmap 051 K3): sim_vehicle_spec's
//! `carla_physics.yaml` onto `VehiclePhysicsControl`, and the read-back that proves it took.
//! The world server applies it to every vehicle a vehicle stack drives (the ego, `agent`
//! vehicles) whose body is the file's blueprint; acb_bridge reads it back on attach.
//!
//! CARLA 0.9.16 puts a blueprint's own tire friction (3.5 for the Model 3) back on live
//! vehicles of that blueprint whenever another vehicle of it spawns, by any client, at once
//! (swb b1c0684; measured 2026-11-15: the car whose physics was applied last loses it, the
//! rest of the physics stays, and a sensor, a walker or another blueprint's vehicle resets
//! nothing). So the physics is kept, not only put on once:
//! [`SpawnWatch`] says when an actor has appeared, and the backend reads its spec'd cars
//! back then and re-applies the file to any that lost it.
//!
//! Why the car applied last, and why re-applying is the wrong cure (measured 2026-11-16,
//! `ux/spawnfix`): `apply_physics_control` gives the blueprint's wheel class a new tire config
//! and rebuilds the car's PhysX vehicle on it, so the car shares that one tire config object
//! with the blueprint; a new car of the blueprint takes it too and sets its friction to the
//! blueprint's, on the old car as well, in place. And the rebuild starts the PhysX vehicle at
//! rest: a car at 11 m/s stood still the next tick (demo incident i7), and even with its
//! velocity set back, its wheels and engine start at 0 rpm, so any brake locks the wheels
//! (a 9.6 m/s^2 skid at brake 0.05). So [`apply_kept`] does both halves: a decoy of the
//! blueprint takes the blueprint's tire config after the car's apply (then no spawn
//! reaches the car at all), and the car's velocity is set back after it. Measured with a
//! decoy whose apply is read back before it is destroyed: 8 runs of 8 kept the car's tires
//! through a spawn, its speed continuous.

use carla::client::{ActorBase, Vehicle, World};
use carla::geom::{Location, Rotation, Transform, Vector2D, Vector3D};
use carla::rpc::{GearPhysicsControl, VehiclePhysicsControl};
use carla_physics_spec::{CarlaPhysics, ReadBack};

/// `control` with the file's physics: what it says, the rest of the blueprint's kept.
pub fn set(control: &mut VehiclePhysicsControl, p: &CarlaPhysics) {
    let v = &p.vehicle;
    control.mass = v.mass as f32;
    control.center_of_mass = Location {
        x: v.center_of_mass[0] as f32,
        y: v.center_of_mass[1] as f32,
        z: v.center_of_mass[2] as f32,
    };
    control.drag_coefficient = v.drag_coefficient as f32;
    control.max_rpm = v.max_rpm as f32;
    control.torque_curve = v
        .torque_curve
        .iter()
        .map(|[rpm, nm]| Vector2D {
            x: *rpm as f32,
            y: *nm as f32,
        })
        .collect();
    control.final_ratio = v.final_ratio as f32;
    control.forward_gears = v
        .forward_gears
        .iter()
        .map(|g| GearPhysicsControl {
            ratio: g.ratio as f32,
            down_ratio: g.down_ratio as f32,
            up_ratio: g.up_ratio as f32,
        })
        .collect();
    control.use_gear_autobox = v.use_gear_autobox;
    control.moi = v.moi as f32;
    control.damping_rate_full_throttle = v.damping_rate_full_throttle as f32;
    control.damping_rate_zero_throttle_clutch_engaged =
        v.damping_rate_zero_throttle_clutch_engaged as f32;
    control.damping_rate_zero_throttle_clutch_disengaged =
        v.damping_rate_zero_throttle_clutch_disengaged as f32;
    for (wheel, w) in control.wheels.iter_mut().zip(&p.wheels) {
        wheel.tire_friction = w.tire_friction as f32;
        wheel.damping_rate = w.damping_rate as f32;
        wheel.max_steer_angle = w.max_steer_angle_deg as f32;
        wheel.radius = (w.radius_m * 100.0) as f32; // CARLA: cm
        wheel.max_brake_torque = w.max_brake_torque as f32;
        wheel.max_handbrake_torque = w.max_handbrake_torque as f32;
        wheel.lat_stiff_max_load = w.lat_stiff_max_load as f32;
        wheel.lat_stiff_value = w.lat_stiff_value as f32;
        wheel.long_stiff_value = w.long_stiff_value as f32;
    }
}

/// What an actor's physics control says, in the file's units.
pub fn read_back(control: &VehiclePhysicsControl) -> ReadBack {
    ReadBack {
        mass: f64::from(control.mass),
        center_of_mass: [
            f64::from(control.center_of_mass.x),
            f64::from(control.center_of_mass.y),
            f64::from(control.center_of_mass.z),
        ],
        drag_coefficient: f64::from(control.drag_coefficient),
        max_rpm: f64::from(control.max_rpm),
        final_ratio: f64::from(control.final_ratio),
        peak_torque: control
            .torque_curve
            .iter()
            .map(|p| f64::from(p.y))
            .fold(0.0, f64::max),
        moi: f64::from(control.moi),
        damping_rate_zero_throttle_clutch_engaged: f64::from(
            control.damping_rate_zero_throttle_clutch_engaged,
        ),
        wheels: control
            .wheels
            .iter()
            .map(|w| {
                [
                    f64::from(w.tire_friction),
                    f64::from(w.radius) / 100.0,
                    f64::from(w.max_brake_torque),
                    f64::from(w.max_steer_angle),
                    f64::from(w.long_stiff_value),
                ]
            })
            .collect(),
    }
}

/// Put the file's physics on `vehicle` ([`apply_kept`]). CARLA applies it at the next tick:
/// [`check`] after. The decoy's id, if one spawned.
pub fn apply(world: &mut World, vehicle: &Vehicle, p: &CarlaPhysics) -> eyre::Result<Option<u32>> {
    let mut control = vehicle
        .physics_control()
        .map_err(|e| eyre::eyre!("read its physics: {e}"))?;
    if control.wheels.len() != p.wheels.len() {
        eyre::bail!(
            "it has {} wheels, the physics {}",
            control.wheels.len(),
            p.wheels.len()
        );
    }
    set(&mut control, p);
    apply_kept(world, vehicle, &control)
}

/// Where a decoy spawns: under every town, in no sensor's view; it never sees a tick.
const DECOY_AT: Location = Location {
    x: 0.0,
    y: 0.0,
    z: -500.0,
};

/// A decoy's role: an entity mark, so a world server that died between its spawn and its
/// destroy leaves an orphan the next one reaps.
pub const DECOY_ROLE: &str = "swb_entity:~tire_decoy";

/// A car's motion, read before an apply rebuilds its PhysX vehicle at rest.
#[derive(Debug, Clone, Copy)]
pub struct Motion {
    pub velocity: Vector3D,
    pub angular_velocity: Vector3D,
}

impl Motion {
    pub fn of(vehicle: &Vehicle) -> eyre::Result<Self> {
        Ok(Self {
            velocity: vehicle
                .velocity()
                .map_err(|e| eyre::eyre!("read its velocity: {e}"))?,
            angular_velocity: vehicle
                .angular_velocity()
                .map_err(|e| eyre::eyre!("read its angular velocity: {e}"))?,
        })
    }

    /// Whether there is anything to set back (a car at rest keeps rest by itself).
    pub fn moving(&self) -> bool {
        let n = |v: &Vector3D| (v.x * v.x + v.y * v.y + v.z * v.z).sqrt();
        n(&self.velocity) > 1e-3 || n(&self.angular_velocity) > 1e-3
    }

    pub fn restore(&self, vehicle: &Vehicle) -> eyre::Result<()> {
        vehicle
            .set_target_velocity(&self.velocity)
            .map_err(|e| eyre::eyre!("set its velocity back: {e}"))?;
        vehicle
            .set_target_angular_velocity(&self.angular_velocity)
            .map_err(|e| eyre::eyre!("set its angular velocity back: {e}"))
    }
}

/// The tires of the decoy's own control are marked this much lower, so its apply can be
/// seen to have happened. No live car ever runs the mark: a spawn sets the tire config it
/// takes to the blueprint's friction.
const DECOY_MARK: f32 = 0.01;

/// Read `vehicle` back until it reports these tires (LibCarla sends an apply without
/// waiting for it, and a destroy sent next overtook it: a decoy destroyed before its apply
/// left the car on the blueprint's tire config, 1 run in 3). Whether it did, within 0.2 s.
fn wait_for_tires(vehicle: &Vehicle, want: &[f32]) -> bool {
    for _ in 0..100 {
        if let Ok(c) = vehicle.physics_control() {
            if tires_are(&c, want) {
                return true;
            }
        }
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
    false
}

fn tires_are(control: &VehiclePhysicsControl, want: &[f32]) -> bool {
    control.wheels.len() == want.len()
        && control
            .wheels
            .iter()
            .zip(want)
            .all(|(w, f)| (w.tire_friction - f).abs() < 1e-3)
}

/// `control` on `vehicle`, which keeps its motion and keeps it alone: a decoy of its
/// blueprint spawns first (a spawn of the blueprint resets the tire config the blueprint
/// shares, so it must come before), `control` goes on the car, the decoy's own control
/// (marked, [`DECOY_MARK`]) on the decoy -- the blueprint's tire config is the decoy's from
/// then on -- and once the decoy reads its mark back it is destroyed, before any tick; then
/// the car's velocity is set back, in the same tick. Returns the decoy's id. Without a
/// decoy (its spawn refused, its apply not seen) the apply still happens, and the next
/// spawn of the blueprint resets this car: the backend's keep re-applies.
pub fn apply_kept(
    world: &mut World,
    vehicle: &Vehicle,
    control: &VehiclePhysicsControl,
) -> eyre::Result<Option<u32>> {
    let motion = Motion::of(vehicle)
        .map_err(|e| tracing::debug!("actor {}: {e:#}", vehicle.id()))
        .ok();
    let blueprint = vehicle.type_id();
    let decoy = match world
        .actor_builder(&blueprint)
        .and_then(|b| b.set_attribute("role_name", DECOY_ROLE))
        .and_then(|b| {
            b.spawn_vehicle(Transform {
                location: DECOY_AT,
                rotation: Rotation::identity(),
            })
        }) {
        Ok(d) => Some(d),
        Err(e) => {
            tracing::warn!(
                "no tire decoy of {blueprint} ({e}): actor {} keeps the blueprint's tire \
                 config, and the next {blueprint} to spawn resets its tires",
                vehicle.id()
            );
            None
        }
    };
    vehicle
        .apply_physics_control(control)
        .map_err(|e| eyre::eyre!("apply the physics: {e}"))?;
    let want: Vec<f32> = control.wheels.iter().map(|w| w.tire_friction).collect();
    if !wait_for_tires(vehicle, &want) {
        tracing::warn!(
            "actor {}: its physics did not read back within 0.2 s of the apply",
            vehicle.id()
        );
    }
    let decoy_id = decoy.map(|d| {
        let id = d.id();
        match d.physics_control() {
            Ok(mut own) => {
                for w in own.wheels.iter_mut() {
                    w.tire_friction -= DECOY_MARK;
                }
                let mark: Vec<f32> = own.wheels.iter().map(|w| w.tire_friction).collect();
                match d.apply_physics_control(&own) {
                    Ok(()) if wait_for_tires(&d, &mark) => {}
                    Ok(()) => tracing::warn!(
                        "tire decoy {id}: its apply did not read back within 0.2 s; actor {} \
                         may keep the blueprint's tire config",
                        vehicle.id()
                    ),
                    Err(e) => tracing::warn!("tire decoy {id}: apply its own physics: {e}"),
                }
            }
            Err(e) => tracing::warn!("tire decoy {id}: read its physics: {e}"),
        }
        if let Err(e) = d.destroy() {
            tracing::warn!("tire decoy {id}: destroy: {e}");
        }
        id
    });
    if let Some(m) = motion.filter(Motion::moving) {
        m.restore(vehicle)?;
    }
    Ok(decoy_id)
}

/// `control` with the file's physics and the road's factor on its tires: what a spec'd car
/// should run on a road whose friction the scenario scaled.
pub fn set_on_road(control: &mut VehiclePhysicsControl, p: &CarlaPhysics, road_factor: f64) {
    set(control, p);
    for wheel in control.wheels.iter_mut() {
        wheel.tire_friction *= road_factor as f32;
    }
}

/// The tires a car with `current` ones and its `own` should run on a road of `factor`, if
/// they are not what it runs already (None: nothing to apply).
pub fn road_tires(current: &[f32], own: &[f32], factor: f64) -> Option<Vec<f32>> {
    let want: Vec<f32> = own.iter().map(|o| o * factor as f32).collect();
    let same =
        current.len() == want.len() && current.iter().zip(&want).all(|(c, w)| (c - w).abs() < 1e-3);
    (!same).then_some(want)
}

/// What `vehicle` runs that is not the file's on this road (empty: all of it is). Read
/// back now: CARLA reports a re-applied or a reset control at once, without a tick.
pub fn off_road(
    vehicle: &Vehicle,
    p: &CarlaPhysics,
    road_factor: f64,
) -> eyre::Result<Vec<String>> {
    let back = vehicle
        .physics_control()
        .map_err(|e| eyre::eyre!("read its physics: {e}"))?;
    Ok(p.differences(&read_back(&back), road_factor))
}

/// Put the file's physics, on this road, back on `vehicle` ([`apply_kept`]: its motion
/// kept). The decoy's id, if one spawned.
pub fn reapply(
    world: &mut World,
    vehicle: &Vehicle,
    p: &CarlaPhysics,
    road_factor: f64,
) -> eyre::Result<Option<u32>> {
    let mut control = vehicle
        .physics_control()
        .map_err(|e| eyre::eyre!("read its physics: {e}"))?;
    set_on_road(&mut control, p, road_factor);
    apply_kept(world, vehicle, &control)
}

/// When an actor has appeared in the world, from the highest actor id seen (CARLA's ids
/// only increase within a server's run). Fed every tick from the world snapshot -- which
/// lists every client's actors, the vehicle bridges' sensors and the Traffic Manager's too
/// -- and from each spawn of the backend's own.
#[derive(Debug, Default)]
pub struct SpawnWatch {
    highest: u32,
}

impl SpawnWatch {
    /// `highest`, the highest actor id in the world now: whether anything spawned since the
    /// last call. A lower one is a new server (or a new watch): it fires too.
    pub fn saw(&mut self, highest: u32) -> bool {
        let fired = highest != self.highest;
        self.highest = highest;
        fired
    }

    /// An actor of ours with this id spawned and was looked after already.
    pub fn spawned(&mut self, id: u32) {
        self.highest = self.highest.max(id);
    }
}

/// After a tick: the physics `vehicle` runs is the file's. An error names what is not.
pub fn check(vehicle: &Vehicle, p: &CarlaPhysics) -> eyre::Result<()> {
    let back = vehicle
        .physics_control()
        .map_err(|e| eyre::eyre!("read the physics back: {e}"))?;
    let off = p.differences(&read_back(&back), 1.0);
    if !off.is_empty() {
        eyre::bail!("the physics did not take: {}", off.join("; "));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_new_actor_fires_the_watch_once() {
        let mut w = SpawnWatch::default();
        assert!(w.saw(40), "the first look at a world fires");
        assert!(!w.saw(40), "nothing new");
        assert!(w.saw(43), "three sensors attached");
        assert!(!w.saw(43));
    }

    #[test]
    fn our_own_spawn_does_not_fire_again() {
        let mut w = SpawnWatch::default();
        w.saw(40);
        w.spawned(41);
        assert!(!w.saw(41), "kept at the spawn already");
        assert!(w.saw(42), "someone else's");
    }

    #[test]
    fn a_car_at_rest_has_no_motion_to_set_back() {
        let still = Motion {
            velocity: Vector3D::new(0.0, 0.0, 0.0),
            angular_velocity: Vector3D::new(0.0, 0.0, 0.0),
        };
        assert!(!still.moving());
        let driving = Motion {
            velocity: Vector3D::new(11.0, 0.2, 0.0),
            ..still
        };
        assert!(driving.moving(), "an 11 m/s car is set back to 11 m/s");
        let turning = Motion {
            angular_velocity: Vector3D::new(0.0, 0.0, 12.0),
            ..still
        };
        assert!(turning.moving());
    }

    #[test]
    fn road_tires_apply_only_when_they_change() {
        let own = [1.2_f32; 4];
        assert_eq!(road_tires(&own, &own, 1.0), None, "on the road's already");
        let wet = road_tires(&own, &own, 0.5).expect("a wet road scales them");
        assert!(wet.iter().all(|w| (w - 0.6).abs() < 1e-6));
        assert_eq!(
            road_tires(&wet, &own, 0.5),
            None,
            "and then they are the road's"
        );
        assert!(
            road_tires(&[3.5; 4], &own, 0.5).is_some(),
            "a reset car gets them back"
        );
    }

    #[test]
    fn a_new_server_fires() {
        let mut w = SpawnWatch::default();
        w.saw(500);
        assert!(w.saw(30), "ids start again on a restarted CARLA");
        assert!(!w.saw(30));
    }
}
