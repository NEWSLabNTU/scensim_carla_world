//! `COMMANDED` bodies (092 K12): a behaviour model on the truth stream commands an entity, and
//! this server moves it. Each command (speed, heading, acceleration limit, in SSv2's frame)
//! reaches the tick gate with the follower's acknowledgement of the frame it answers
//! ([`crate::tick_gate`]); before each step's first tick the body integrates its latest command
//! over the step, as the batch does (`scensim_batch.core.Models`, the same dt): the speed moves
//! toward the commanded speed by at most the limit times dt, the yaw is set to the heading, and
//! the position moves `v dt` along it. The
//! actor is then posed with physics off. CARLA reports a physics-off actor at 0 m/s, so what
//! SSv2 is told (twist, acceleration, jerk) and the step's states report (the velocity, scensim
//! issue 160) are this state, not CARLA's.

use crate::tick_gate::Command;

/// One commanded body's state, in SSv2's map frame (m, rad, m/s, m/s2, m/s3).
#[derive(Debug, Clone, PartialEq)]
pub struct Body {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub yaw: f64,
    pub v: f64,
    pub accel: f64,
    pub jerk: f64,
    /// The gate frame whose command moved it last; 0 before its first command.
    pub cmd_frame: u64,
}

impl Body {
    /// At rest where it spawned.
    pub fn at(x: f64, y: f64, z: f64, yaw: f64) -> Self {
        Self {
            x,
            y,
            z,
            yaw,
            v: 0.0,
            accel: 0.0,
            jerk: 0.0,
            cmd_frame: 0,
        }
    }

    /// Its velocity in the map frame, m/s: the speed along the yaw. What the step's states
    /// report for it (scensim issue 160), not CARLA's 0 for a teleported actor.
    pub fn velocity(&self) -> [f64; 3] {
        [self.v * self.yaw.cos(), self.v * self.yaw.sin(), 0.0]
    }

    /// One tick of `dt` seconds under `command`, which answered gate frame `frame`.
    pub fn step(&mut self, command: &Command, frame: u64, dt: f64) {
        let v0 = self.v;
        let lim = command.accel_limit * dt;
        self.v = v0 + (command.speed - v0).clamp(-lim, lim);
        self.yaw = command.heading;
        self.x += self.v * command.heading.cos() * dt;
        self.y += self.v * command.heading.sin() * dt;
        let accel = (self.v - v0) / dt;
        self.jerk = (accel - self.accel) / dt;
        self.accel = accel;
        self.cmd_frame = frame;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd(speed: f64, heading: f64, accel_limit: f64) -> Command {
        Command {
            entity: "walker".into(),
            speed,
            heading,
            accel_limit,
        }
    }

    /// The batch's law, step for step: `scensim_batch.core.Models.step` on float64 (its lines
    /// `v = v + max(min(speed - v, lim), -lim)`, `yaw = heading`, `x += v cos(heading) dt`,
    /// `y += v sin(heading) dt`), computed here independently as the reference.
    #[test]
    fn the_law_is_the_batchs() {
        let dt = 0.05;
        let commands = [
            cmd(1.4, 0.3, 1.0),
            cmd(1.4, 0.3, 1.0),
            cmd(0.0, 0.3, 2.5),
            cmd(1.4, -1.2, 0.8),
            cmd(1.4, -1.2, 0.8),
        ];
        let mut b = Body::at(10.0, -3.0, 0.0, 0.0);
        let (mut x, mut y, mut v) = (10.0_f64, -3.0_f64, 0.0_f64);
        for (k, c) in commands.iter().enumerate() {
            b.step(c, 100 + k as u64, dt);
            let lim = c.accel_limit * dt;
            v += (c.speed - v).min(lim).max(-lim);
            x += v * c.heading.cos() * dt;
            y += v * c.heading.sin() * dt;
            assert_eq!((b.x, b.y, b.v, b.yaw), (x, y, v, c.heading), "step {k}");
            assert_eq!(b.cmd_frame, 100 + k as u64);
        }
    }

    /// 092 K12's exit check 1: the batch's law (`scensim_batch.core.model_law`, which wrote
    /// `commanded_law.json`; scensim_batch's test_model_law checks the file against it) and
    /// this integrator, on the same commands, agree within float64 rounding.
    #[test]
    fn the_integrator_matches_the_batchs_law_on_its_fixture() {
        let f: serde_json::Value =
            serde_json::from_str(include_str!("commanded_law.json")).expect("the fixture");
        let dt = f["dt"].as_f64().unwrap();
        let start = f["start"].as_array().unwrap();
        let mut b = Body::at(
            start[0].as_f64().unwrap(),
            start[1].as_f64().unwrap(),
            0.0,
            0.0,
        );
        let commands = f["commands"].as_array().unwrap();
        let states = f["states"].as_array().unwrap();
        assert_eq!(commands.len(), states.len());
        assert!(commands.len() >= 20);
        let close = |a: f64, b: f64| (a - b).abs() <= 1e-12 * a.abs().max(b.abs()).max(1.0);
        for (k, (c, want)) in commands.iter().zip(states).enumerate() {
            let n = |v: &serde_json::Value, i: usize| v[i].as_f64().unwrap();
            b.step(&cmd(n(c, 0), n(c, 1), n(c, 2)), k as u64 + 1, dt);
            let got = [b.x, b.y, b.v, b.yaw];
            for (i, g) in got.iter().enumerate() {
                assert!(
                    close(*g, n(want, i)),
                    "step {k} field {i}: {g} vs {}",
                    n(want, i)
                );
            }
        }
    }

    #[test]
    fn speed_moves_within_the_limit_and_settles_on_the_command() {
        let mut b = Body::at(0.0, 0.0, 0.0, 0.0);
        b.step(&cmd(1.0, 0.0, 2.0), 1, 0.1);
        assert!((b.v - 0.2).abs() < 1e-12 && (b.accel - 2.0).abs() < 1e-9);
        for f in 2..20 {
            b.step(&cmd(1.0, 0.0, 2.0), f, 0.1);
        }
        assert_eq!(b.v, 1.0);
        assert_eq!(b.accel, 0.0);
        assert!(b.x > 1.0 && b.y == 0.0);
    }
}
