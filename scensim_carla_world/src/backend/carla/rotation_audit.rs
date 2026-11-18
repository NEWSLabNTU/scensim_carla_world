//! Rotation-order audit for CARLA 1.0 (roadmap 083 step 4).
//!
//! 1.0's changelog says the pitch and roll rotation order was corrected. That fix is in
//! LibCarla's client-side math (`geom::Rotation::RotateVector`, the forward/right/up
//! vectors), which now follows Unreal's FRotator: +pitch tilts the forward axis up, +roll
//! tilts the right axis down. The server always applied FRotator to actors.
//!
//! This world server never calls that math. It converts poses itself
//! (`coordinate_conversion::carla_to_ros_rotation` + `euler_to_quaternion`, and back), so
//! what has to hold is that our conversion is FRotator's, Y-flipped into ROS. The tests
//! below check exactly that from known poses, against FRotator's matrix written out here
//! (Unreal's `FRotationMatrix::MakeFromRotator`, independent of carla-rust), and against the
//! linked LibCarla's `Rotation::rotate_vector`, on 0.9.16 and 1.0 builds alike. Result
//! (2026-11-18, carla-rust 1212b60): all three agree on both versions, from every pose
//! below; no compensation for the old order exists in this code, and none is needed.

#[cfg(test)]
mod tests {
    use crate::coordinate_conversion::{
        carla_to_ros_rotation, euler_to_quaternion, quaternion_to_euler, ros_to_carla_rotation,
    };

    type V = [f64; 3];

    /// Unreal's rotation matrix rows for FRotator(pitch, yaw, roll) in degrees: the actor's
    /// forward (X), right (Y) and up (Z) axes in CARLA's left-handed world frame.
    fn frotator_axes(pitch: f64, yaw: f64, roll: f64) -> (V, V, V) {
        let (sp, cp) = pitch.to_radians().sin_cos();
        let (sy, cy) = yaw.to_radians().sin_cos();
        let (sr, cr) = roll.to_radians().sin_cos();
        let forward = [cp * cy, cp * sy, sp];
        let right = [sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp];
        let up = [-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp];
        (forward, right, up)
    }

    /// CARLA (left-handed, y right) → ROS (right-handed, y left).
    fn flip(v: V) -> V {
        [v[0], -v[1], v[2]]
    }

    /// Rotate `v` by the unit quaternion (x, y, z, w).
    fn rotate(q: (f64, f64, f64, f64), v: V) -> V {
        let (x, y, z, w) = q;
        // v' = v + 2w(u×v) + 2u×(u×v), u = (x, y, z)
        let u = [x, y, z];
        let cross = |a: V, b: V| -> V {
            [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]
        };
        let t = cross(u, v);
        let t2 = cross(u, t);
        [
            v[0] + 2.0 * (w * t[0] + t2[0]),
            v[1] + 2.0 * (w * t[1] + t2[1]),
            v[2] + 2.0 * (w * t[2] + t2[2]),
        ]
    }

    fn close(a: V, b: V, what: &str) {
        let d = ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
        assert!(d < 1e-9, "{what}: {a:?} != {b:?}");
    }

    /// Known poses: each axis alone, then combined ones, signs both ways.
    const POSES: [(f64, f64, f64); 7] = [
        (20.0, 0.0, 0.0),
        (0.0, 30.0, 0.0),
        (0.0, 0.0, 25.0),
        (20.0, 30.0, 10.0),
        (-15.0, 135.0, -40.0),
        (5.0, -170.0, 3.0),
        (-60.0, 75.0, 80.0),
    ];

    /// The ROS quaternion we report for a CARLA pose.
    fn ros_quaternion(pitch: f64, yaw: f64, roll: f64) -> (f64, f64, f64, f64) {
        let (r, p, y) = carla_to_ros_rotation(roll, pitch, yaw);
        euler_to_quaternion(r, p, y)
    }

    #[test]
    fn carla_to_ros_is_frotator_flipped() {
        for (pitch, yaw, roll) in POSES {
            let (fwd, right, up) = frotator_axes(pitch, yaw, roll);
            let q = ros_quaternion(pitch, yaw, roll);
            let pose = format!("pitch {pitch} yaw {yaw} roll {roll}");
            close(
                rotate(q, [1.0, 0.0, 0.0]),
                flip(fwd),
                &format!("forward, {pose}"),
            );
            // ROS +y is the body's left: CARLA's right, negated, flipped
            close(
                rotate(q, [0.0, 1.0, 0.0]),
                flip([-right[0], -right[1], -right[2]]),
                &format!("left, {pose}"),
            );
            close(rotate(q, [0.0, 0.0, 1.0]), flip(up), &format!("up, {pose}"));
        }
    }

    #[test]
    fn the_unreal_semantics_hold_in_ros() {
        // +pitch: the nose goes up
        let q = ros_quaternion(20.0, 0.0, 0.0);
        assert!(rotate(q, [1.0, 0.0, 0.0])[2] > 0.3);
        // +roll: the right side goes down, so the body's left (ROS +y) goes up
        let q = ros_quaternion(0.0, 0.0, 25.0);
        assert!(rotate(q, [0.0, 1.0, 0.0])[2] > 0.4);
        // +yaw (clockwise from above in CARLA): the nose turns to the ROS right (-y)
        let q = ros_quaternion(0.0, 30.0, 0.0);
        assert!(rotate(q, [1.0, 0.0, 0.0])[1] < -0.4);
    }

    #[test]
    fn ros_to_carla_round_trips() {
        for (pitch, yaw, roll) in POSES {
            let (x, y, z, w) = ros_quaternion(pitch, yaw, roll);
            let (r, p, yw) = quaternion_to_euler(x, y, z, w);
            let (roll2, pitch2, yaw2) = ros_to_carla_rotation(r, p, yw);
            // compare as rotations (Euler angles alias near the poles and at +-180)
            let (a, b, c) = frotator_axes(pitch, yaw, roll);
            let (a2, b2, c2) = frotator_axes(pitch2, yaw2, roll2);
            let pose = format!("pitch {pitch} yaw {yaw} roll {roll}");
            close(a, a2, &format!("forward, {pose}"));
            close(b, b2, &format!("right, {pose}"));
            close(c, c2, &format!("up, {pose}"));
        }
    }

    /// LibCarla's own rotation (`Rotation::rotate_vector`, the client the binary links) is
    /// FRotator too, on 0.9.16 and on 1.0 alike, so it agrees with ours from every pose.
    /// carla-rust's `Rotation::to_na()` is not used here or in the conversion: it disagrees
    /// with LibCarla (an old carla-rust issue, not a 1.0 one).
    #[test]
    fn libcarla_rotate_vector_agrees() {
        use carla::geom::{Rotation, Vector3D};
        for (pitch, yaw, roll) in POSES {
            let r = Rotation {
                pitch: pitch as f32,
                yaw: yaw as f32,
                roll: roll as f32,
            };
            let (fwd, right, up) = frotator_axes(pitch, yaw, roll);
            for (axis, want) in [
                ([1.0, 0.0, 0.0], fwd),
                ([0.0, 1.0, 0.0], right),
                ([0.0, 0.0, 1.0], up),
            ] {
                let got = r.rotate_vector(&Vector3D::new(
                    axis[0] as f32,
                    axis[1] as f32,
                    axis[2] as f32,
                ));
                let got = [f64::from(got.x), f64::from(got.y), f64::from(got.z)];
                let d = (0..3).map(|i| (got[i] - want[i]).abs()).fold(0.0, f64::max);
                assert!(
                    d < 1e-4,
                    "axis {axis:?} pitch {pitch} yaw {yaw} roll {roll}: {got:?} != {want:?}"
                );
            }
        }
    }
}
