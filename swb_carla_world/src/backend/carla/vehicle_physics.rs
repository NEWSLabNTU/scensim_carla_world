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

use carla::client::Vehicle;
use carla::geom::{Location, Vector2D};
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

/// Put the file's physics on `vehicle`. CARLA applies it at the next tick: [`check`] after.
pub fn apply(vehicle: &Vehicle, p: &CarlaPhysics) -> eyre::Result<()> {
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
    vehicle
        .apply_physics_control(&control)
        .map_err(|e| eyre::eyre!("apply the physics: {e}"))
}

/// `control` with the file's physics and the road's factor on its tires: what a spec'd car
/// should run on a road whose friction the scenario scaled.
pub fn set_on_road(control: &mut VehiclePhysicsControl, p: &CarlaPhysics, road_factor: f64) {
    set(control, p);
    for wheel in control.wheels.iter_mut() {
        wheel.tire_friction *= road_factor as f32;
    }
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

/// Put the file's physics, on this road, back on `vehicle`.
pub fn reapply(vehicle: &Vehicle, p: &CarlaPhysics, road_factor: f64) -> eyre::Result<()> {
    let mut control = vehicle
        .physics_control()
        .map_err(|e| eyre::eyre!("read its physics: {e}"))?;
    set_on_road(&mut control, p, road_factor);
    vehicle
        .apply_physics_control(&control)
        .map_err(|e| eyre::eyre!("apply the physics: {e}"))
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
    fn a_new_server_fires() {
        let mut w = SpawnWatch::default();
        w.saw(500);
        assert!(w.saw(30), "ids start again on a restarted CARLA");
        assert!(!w.saw(30));
    }
}
