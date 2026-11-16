//! The town's own movable props, frozen (roadmap 077 G58).
//!
//! Some CARLA towns place physics-simulated props in the level itself: Town01 has plastic
//! chairs (0.5 kg), tables (5 kg) and cones (24 kg) by the road, listed as `static.prop.mesh`
//! actors. Nothing in a scenario declares them, so SSv2 cannot see them, yet any NPC that
//! brushes one knocks it into a lane, where the ego may hit it. A verdict that depends on
//! where a chair rolled is chance. So the world server turns physics off on every such
//! actor it did not spawn, after each world load (at every `LoadWorld`, which also covers a
//! world CARLA reloaded or restarted with): they keep their colliders and stay where the
//! level put them. `map_props: live` keeps them simulated.
//!
//! It is a world-server setting, not a host one, because it changes results.

use std::path::Path;

use eyre::{Result, WrapErr};
use serde::Deserialize;

use crate::actor_census::Found;

/// The CARLA type of a prop placed in the level (and of nothing a client spawns by name).
pub const MAP_PROP_TYPE: &str = "static.prop.mesh";

/// What happens to the level's movable props.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum Mode {
    /// Physics off: they stay where the level put them (the default).
    #[default]
    Frozen,
    /// CARLA's own: simulated, so a car can knock them about.
    Live,
}

impl Mode {
    pub fn name(self) -> &'static str {
        match self {
            Mode::Frozen => "frozen",
            Mode::Live => "live",
        }
    }
}

/// The `map_props` key of `swb_carla_world.yaml`, this server's alone.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(default)]
pub struct Config {
    pub map_props: Mode,
}

impl Config {
    /// The key of `text` (a whole `swb_carla_world.yaml`); every other key is ignored.
    pub fn parse(text: &str) -> Result<Self> {
        if text.trim().is_empty() {
            return Ok(Self::default());
        }
        Ok(serde_yaml::from_str(text)?)
    }

    /// The key from the file `path`; a missing file is the default, a malformed one an error.
    pub fn load(path: &Path) -> Result<Self> {
        if !path.exists() {
            return Ok(Self::default());
        }
        let text =
            std::fs::read_to_string(path).wrap_err_with(|| format!("read {}", path.display()))?;
        Self::parse(&text).wrap_err_with(|| format!("parse {}", path.display()))
    }
}

/// The level's props among `found`: every `static.prop.mesh` that this server did not spawn
/// (`own`) and that carries no swb entity mark (`mark_prefix`, a previous server's).
pub fn to_freeze(found: &[Found], mark_prefix: &str, own: impl Fn(u32) -> bool) -> Vec<u32> {
    found
        .iter()
        .filter(|f| f.type_id == MAP_PROP_TYPE)
        .filter(|f| !own(f.id))
        .filter(|f| {
            !f.role
                .as_deref()
                .is_some_and(|r| r.starts_with(mark_prefix))
        })
        .map(|f| f.id)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn actor(id: u32, type_id: &str, role: Option<&str>) -> Found {
        Found {
            id,
            type_id: type_id.into(),
            role: role.map(str::to_owned),
            parent: None,
        }
    }

    #[test]
    fn map_props_are_frozen_and_swb_props_are_left_alone() {
        let world = vec![
            // Town01's own: a chair, a table, a cone.
            actor(130, MAP_PROP_TYPE, None),
            actor(131, MAP_PROP_TYPE, None),
            actor(173, MAP_PROP_TYPE, None),
            // The rest of the level: not props with physics.
            actor(2, "traffic.traffic_light", None),
            actor(3, "static.trigger.traffic_sign", None),
            // Spawned by this server: a misc object, and one that happens to be a mesh.
            actor(500, "static.prop.streetbarrier", Some("swb_entity:cone_1")),
            actor(501, MAP_PROP_TYPE, Some("swb_entity:box_1")),
            actor(502, MAP_PROP_TYPE, None),
            // A previous server's, marked: the reaper's, not ours to freeze.
            actor(400, MAP_PROP_TYPE, Some("swb_entity:old")),
            // A vehicle.
            actor(600, "vehicle.tesla.model3", Some("hero")),
        ];
        let own = |id: u32| id >= 500;
        assert_eq!(to_freeze(&world, "swb_entity:", own), vec![130, 131, 173]);
    }

    #[test]
    fn frozen_is_the_default_and_live_keeps_them_simulated() {
        assert_eq!(Config::parse("").unwrap().map_props, Mode::Frozen);
        assert_eq!(
            Config::parse("spectator: off\ncarla: {port: 2000}\n")
                .unwrap()
                .map_props,
            Mode::Frozen
        );
        assert_eq!(
            Config::parse("map_props: live\n").unwrap().map_props,
            Mode::Live
        );
        assert_eq!(
            Config::parse("map_props: frozen\n").unwrap().map_props,
            Mode::Frozen
        );
        assert!(Config::parse("map_props: melted\n").is_err());
    }
}
