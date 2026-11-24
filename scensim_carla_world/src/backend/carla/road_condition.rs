//! OpenSCENARIO's road condition on CARLA (roadmap 047, S2): what CARLA 0.9.16 can do with it.
//!
//! CARLA has one friction per wheel (`WheelPhysicsControl::tire_friction`) and no road
//! surface model, so the condition becomes a factor on every vehicle's tire friction:
//! `frictionScaleFactor` times the wet factor of asphalt, the class CARLA's towns are paved
//! with (the same numbers as `isaac_world/config/surface_classes.yaml`, which the tests
//! check). The wetness also changes the weather's look: `wetness` and
//! `precipitation_deposits` (puddles), which on 0.9.16 are rendering only. Standing water,
//! hydroplaning and the map's zones (`scensim.zone.*`) are the Isaac world server's surface
//! layer; their properties are refused here.

use super::weather::Weather;
use scensim_core::sim::RoadCondition;

/// Asphalt's friction factor per wetness level, dry first: `surface_classes.yaml`'s
/// `classes.asphalt.wet`.
pub const WET_FRICTION: [(&str, f64); 5] = [
    ("dry", 1.0),
    ("moist", 0.9),
    ("wetWithPuddles", 0.75),
    ("lowFlooded", 0.65),
    ("highFlooded", 0.55),
];

/// The weather's `wetness` and `precipitation_deposits` (0..100) per wetness level: how wet
/// the road looks. Chosen by eye, not measured.
pub const WET_LOOK: [(&str, f32, f32); 5] = [
    ("dry", 0.0, 0.0),
    ("moist", 40.0, 0.0),
    ("wetWithPuddles", 80.0, 40.0),
    ("lowFlooded", 100.0, 80.0),
    ("highFlooded", 100.0, 100.0),
];

/// What CARLA makes of a road condition.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Applied {
    /// Every vehicle's tire friction is its own times this.
    pub friction_factor: f64,
    /// The weather's `wetness`, `precipitation_deposits`.
    pub look: (f32, f32),
}

/// The road condition as CARLA applies it, or why not (every problem, one per line).
pub fn applied(rc: &RoadCondition) -> Result<Applied, String> {
    let mut problems = Vec::new();
    let refused: Vec<&str> = rc.scensim_properties().map(|(k, _)| k.as_str()).collect();
    if !refused.is_empty() {
        problems.push(format!(
            "CARLA 0.9.16 cannot apply surface zones: its friction trigger acts on every car of \
             a blueprint, and a per-wheel physics apply steps the car 0.14 to 0.15 m/s (scensim \
             roadmap 086 C7's P0 and P0b); zones and other scensim properties are the Isaac \
             world server's (refused: {})",
            refused.join(", ")
        ));
    }
    let wetness = if rc.wetness.is_empty() {
        "dry"
    } else {
        rc.wetness.as_str()
    };
    let wet = WET_FRICTION.iter().find(|(w, _)| *w == wetness);
    let look = WET_LOOK.iter().find(|(w, _, _)| *w == wetness);
    let (Some((_, wet)), Some((_, wetness_look, deposits))) = (wet, look) else {
        problems.push(format!("unknown wetness {wetness:?}"));
        return Err(problems.join("\n"));
    };
    let f = rc.friction_scale_factor;
    if !(f.is_finite() && f >= 0.0) {
        problems.push(format!("frictionScaleFactor {f} is not in [0, inf)"));
    }
    if !problems.is_empty() {
        return Err(problems.join("\n"));
    }
    Ok(Applied {
        friction_factor: f * wet,
        look: (*wetness_look, *deposits),
    })
}

/// The world-wide road condition CARLA applies, read back (086 C7, `ROAD_CONDITION_APPLIED`):
/// `factor` the cars' tyre friction over their own, `look` the weather's `wetness` and
/// `precipitation_deposits`. The wetness is the level whose look the weather shows (within
/// 0.5), the scale `factor` over that level's asphalt factor. A look that is no level's is
/// refused, naming it.
pub fn read_back(factor: f64, look: (f32, f32)) -> Result<(f64, String), String> {
    let Some((level, _, _)) = WET_LOOK
        .iter()
        .find(|(_, w, d)| (w - look.0).abs() <= 0.5 && (d - look.1).abs() <= 0.5)
    else {
        return Err(format!(
            "the weather's wetness {} and deposits {} are no wetness level's look",
            look.0, look.1
        ));
    };
    let wet = WET_FRICTION
        .iter()
        .find(|(w, _)| w == level)
        .map(|(_, f)| *f)
        .unwrap_or(1.0);
    Ok((factor / wet, level.to_string()))
}

/// The factor every wheel of a car runs over its own tyre friction, or why they differ.
pub fn wheel_factor(current: &[f32], own: &[f32]) -> Result<f64, String> {
    if current.is_empty() || current.len() != own.len() {
        return Err(format!(
            "{} wheels read against {} own",
            current.len(),
            own.len()
        ));
    }
    let f: Vec<f64> = current
        .iter()
        .zip(own)
        .map(|(c, o)| f64::from(*c) / f64::from(*o))
        .collect();
    if f.iter().any(|x| (x - f[0]).abs() > 1e-3) {
        return Err(format!("the wheels run different factors: {f:?}"));
    }
    Ok(f[0])
}

/// `weather` with the road's wet look.
pub fn with_look(mut weather: Weather, look: (f32, f32)) -> Weather {
    weather.wetness = look.0;
    weather.precipitation_deposits = look.1;
    weather
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn rc(f: f64, wetness: &str, props: &[(&str, &str)]) -> RoadCondition {
        RoadCondition {
            friction_scale_factor: f,
            wetness: wetness.into(),
            properties: props
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect::<BTreeMap<_, _>>(),
        }
    }

    #[test]
    fn friction_is_the_scale_times_wet_asphalt() {
        let a = applied(&rc(0.8, "wetWithPuddles", &[])).unwrap();
        assert!((a.friction_factor - 0.6).abs() < 1e-12);
        assert_eq!(a.look, (80.0, 40.0));
        let a = applied(&RoadCondition::default()).unwrap();
        assert_eq!(a.friction_factor, 1.0);
        assert_eq!(a.look, (0.0, 0.0));
        // Another tool's property is not ours to refuse.
        assert!(applied(&rc(1.0, "", &[("esmini.x", "1")])).is_ok());
    }

    #[test]
    fn zones_are_refused_with_every_problem() {
        let e = applied(&rc(
            -1.0,
            "dry",
            &[
                ("scensim.zone.ford.water_depth", "0.1"),
                ("scensim.zone.pit.class", "mud_wet"),
            ],
        ))
        .unwrap_err();
        assert!(
            e.contains("scensim.zone.ford.water_depth, scensim.zone.pit.class"),
            "{e}"
        );
        assert!(e.contains("CARLA 0.9.16 cannot apply surface zones"), "{e}");
        assert!(e.contains("086 C7"), "{e}");
        assert!(e.contains("frictionScaleFactor -1"), "{e}");
        assert!(applied(&rc(1.0, "soaked", &[]))
            .unwrap_err()
            .contains("soaked"));
    }

    #[test]
    fn the_read_back_inverts_the_apply_at_every_level() {
        for (level, _) in WET_FRICTION {
            for scale in [1.0, 0.8, 0.35] {
                let a = applied(&rc(scale, level, &[])).unwrap();
                // CARLA's tyre friction is f32: the factor as a car reads it back
                let own = 3.5f32;
                let factor = wheel_factor(&[own * a.friction_factor as f32; 4], &[own; 4]).unwrap();
                let (s, w) = read_back(factor, a.look).unwrap();
                assert!((s - scale).abs() < 1e-5, "{level} {scale}: {s}");
                assert_eq!(w, *level);
            }
        }
    }

    #[test]
    fn a_read_back_that_does_not_fit_is_refused() {
        assert!(read_back(1.0, (55.0, 0.0))
            .unwrap_err()
            .contains("no wetness level"));
        assert!(wheel_factor(&[3.5, 2.1, 3.5, 3.5], &[3.5; 4])
            .unwrap_err()
            .contains("different factors"));
        assert!(wheel_factor(&[], &[]).is_err());
        // a car whose friction was set by hand to another factor reads back that factor
        let (s, _) = read_back(wheel_factor(&[2.8; 4], &[3.5; 4]).unwrap(), (0.0, 0.0)).unwrap();
        assert!((s - 0.8).abs() < 1e-6);
    }

    /// The wet factors are asphalt's in the Isaac surface layer's class table, so one
    /// scenario brakes alike on both simulators.
    #[test]
    fn wet_factors_match_the_isaac_class_table() {
        // Isaac's table is scensim_isaac_world's; scensim, which holds both world
        // servers, points SCENSIM_ISAAC_SURFACE_CLASSES at it in its `just test`.
        let Some(path) = std::env::var_os("SCENSIM_ISAAC_SURFACE_CLASSES") else {
            eprintln!(
                "SKIPPED: SCENSIM_ISAAC_SURFACE_CLASSES unset; not compared with Isaac's table"
            );
            return;
        };
        let table: serde_yaml::Value =
            serde_yaml::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        let wet = &table["classes"]["asphalt"]["wet"];
        for (level, factor) in WET_FRICTION.iter().skip(1) {
            assert_eq!(wet[*level].as_f64(), Some(*factor), "{level}");
        }
    }
}
