//! `COMMANDED` bodies (092 K12): a behaviour model on the truth stream commands an entity, and
//! this server moves it. Each command (speed, heading, acceleration limit, in SSv2's frame)
//! reaches the tick gate with the follower's acknowledgement of the frame it answers
//! ([`crate::tick_gate`]); before each tick the body integrates its latest command, as the batch
//! does (`scensim_batch.core.Models`): the speed moves toward the commanded speed by at most the
//! limit times dt, the yaw is set to the heading, and the position moves `v dt` along it. The
//! actor is then posed with physics off. CARLA reports a physics-off actor at 0 m/s, so what
//! SSv2 is told (twist, acceleration, jerk) is this state, not CARLA's.

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
