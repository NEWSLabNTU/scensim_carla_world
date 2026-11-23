//! The tick gate: scensim waits, before each `world.tick()`, until every follower that asked for
//! it has finished the frame scensim ticked last.
//!
//! scensim is the only process that ticks CARLA (docs/design/time-and-ticking.md). A process
//! that follows scensim's frames instead of ticking (2026-vehicle-consensus' junction harness)
//! acts a frame late, or skips frames, whenever one of its rounds takes longer than scensim's
//! frame. With the gate the follower still never ticks, but scensim holds the next tick until the
//! follower says it is done with the current frame, so the follower sees every frame.
//!
//! Several followers can join at once (the harness, and ns-3 emulating the network in
//! lockstep with the world): each is known by the address its datagrams come from, and a
//! tick waits for all of them.
//!
//! Protocol, UDP datagrams to the gate's address (`SCENSIM_TICK_GATE`, or `tick_gate` in
//! `bridge_config.yaml`):
//!
//! - `HELLO` -- a follower joins: from now on every tick waits for it (a `DONE` joins too);
//! - `DONE <frame>` -- the follower finished CARLA frame `<frame>`;
//! - `BYE` -- the follower leaves: ticks no longer wait.
//!
//! A follower that commands entities (092 K12: a behaviour model on the truth stream) appends
//! its commands to `DONE`, so they are here before the tick that follows the frame they answer:
//! `DONE <frame> [<entity> <speed> <heading> <accel_limit>]...` (SSv2's frame: m/s, rad, m/s2).
//! Free-running, it sends the same list as `CMD <frame> ...`, which joins nothing and is never
//! waited for. A datagram is at most [`MAX_DATAGRAM`] bytes (it never fragments on an Ethernet
//! link), a name at most [`MAX_NAME`]; a datagram over either, or one that does not parse, is
//! counted ([`TickGate::malformed`]) and dropped whole, its commands never applied.
//!
//! The gate waits once per SSv2 step, before the step's first tick, for the CARLA frame the
//! previous step ended on: the frame the truth stream reported, the only one a follower sees.
//! The ticks inside a step (its substeps) and the settling tick after each spawn are not
//! waited for ([`StepGate`]): no truth frame names them, so no follower could finish them, and
//! each cost a whole timeout (092 K4's exit runs: four after the spawns of 20 NPCs).
//!
//! The wait is bounded (`tick_gate_timeout_ms`): a follower that does not answer in time does
//! not stop the simulation, the tick goes ahead and the miss is counted, per follower. A follower silent
//! for [`SILENT_LEAVES`] is taken as gone, so a crashed follower costs at most a few
//! bounded waits. The wait runs in scensim's request path before the tick, never inside a tick
//! or a sensor callback.

use std::collections::BTreeMap;
use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

/// A follower silent this long has left (its process died without a `BYE`).
pub const SILENT_LEAVES: Duration = Duration::from_secs(5);

/// The largest datagram the gate takes: an Ethernet MTU less IPv4 and UDP headers.
pub const MAX_DATAGRAM: usize = 1472;

/// The longest entity name a command may carry, in bytes.
pub const MAX_NAME: usize = 64;

/// One entity's kinematic command (092 K12), in SSv2's frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Command {
    pub entity: String,
    /// m/s, at least 0.
    pub speed: f64,
    /// rad, SSv2's (ROS) yaw.
    pub heading: f64,
    /// m/s2, greater than 0.
    pub accel_limit: f64,
}

/// A command and the frame it answered.
#[derive(Debug, Clone, PartialEq)]
pub struct Commanded {
    pub frame: u64,
    pub command: Command,
}

/// One datagram, whole: a gate message (none for `CMD`), and the commands it carries with the
/// frame they answer.
#[derive(Debug, Clone, PartialEq)]
pub struct Datagram {
    pub msg: Option<GateMsg>,
    pub frame: u64,
    pub commands: Vec<Command>,
}

impl Datagram {
    /// The datagram, or why it is refused.
    pub fn parse(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() > MAX_DATAGRAM {
            return Err(format!("{} bytes, over {MAX_DATAGRAM}", bytes.len()));
        }
        let text = std::str::from_utf8(bytes).map_err(|_| "not UTF-8".to_string())?;
        let mut f = text.split_whitespace();
        let head = f.next().ok_or("empty")?;
        let (msg, frame) = match head {
            "HELLO" | "BYE" => {
                if f.next().is_some() {
                    return Err(format!("{head} takes nothing after it"));
                }
                let msg = GateMsg::parse(head).expect("a gate word");
                return Ok(Self {
                    msg: Some(msg),
                    frame: 0,
                    commands: vec![],
                });
            }
            "DONE" | "CMD" => {
                let frame: u64 = f
                    .next()
                    .ok_or(format!("{head} without a frame"))?
                    .parse()
                    .map_err(|_| format!("{head}: the frame is not a number"))?;
                ((head == "DONE").then_some(GateMsg::Done(frame)), frame)
            }
            other => return Err(format!("unknown word {other:?}")),
        };
        let rest: Vec<&str> = f.collect();
        if !rest.len().is_multiple_of(4) {
            return Err(format!(
                "{} field(s) after the frame: commands are 4 each",
                rest.len()
            ));
        }
        let mut commands = Vec::with_capacity(rest.len() / 4);
        for c in rest.chunks(4) {
            let entity = c[0];
            if entity.len() > MAX_NAME {
                return Err(format!("entity name over {MAX_NAME} bytes"));
            }
            if commands.iter().any(|k: &Command| k.entity == entity) {
                return Err(format!("{entity:?} commanded twice"));
            }
            let num = |s: &str, what: &str| -> Result<f64, String> {
                s.parse::<f64>()
                    .ok()
                    .filter(|v| v.is_finite())
                    .ok_or(format!("{entity:?}: {what} {s:?} is not a finite number"))
            };
            let speed = num(c[1], "speed")?;
            let heading = num(c[2], "heading")?;
            let accel_limit = num(c[3], "acceleration limit")?;
            if speed < 0.0 {
                return Err(format!("{entity:?}: speed {speed} is negative"));
            }
            if accel_limit <= 0.0 {
                return Err(format!(
                    "{entity:?}: acceleration limit {accel_limit} is not positive"
                ));
            }
            commands.push(Command {
                entity: entity.to_string(),
                speed,
                heading,
                accel_limit,
            });
        }
        Ok(Self {
            msg,
            frame,
            commands,
        })
    }
}

/// One datagram of the protocol.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateMsg {
    Hello,
    Done(u64),
    Bye,
}

impl GateMsg {
    pub fn parse(text: &str) -> Option<Self> {
        let mut f = text.split_whitespace();
        match f.next()? {
            "HELLO" => Some(Self::Hello),
            "BYE" => Some(Self::Bye),
            "DONE" => f.next()?.parse().ok().map(Self::Done),
            _ => None,
        }
    }
}

/// Whether a follower is attached and the last frame it finished.
#[derive(Debug, Clone, Default)]
pub struct GateState {
    pub active: bool,
    pub done: Option<u64>,
    pub last_heard: Option<Instant>,
}

impl GateState {
    pub fn apply(&mut self, msg: GateMsg, now: Instant) {
        self.last_heard = Some(now);
        match msg {
            GateMsg::Hello => {
                self.active = true;
                self.done = None;
            }
            GateMsg::Done(f) => {
                self.active = true;
                self.done = Some(self.done.map_or(f, |d| d.max(f)));
            }
            GateMsg::Bye => {
                self.active = false;
                self.done = None;
            }
        }
    }

    /// Whether the tick after `frame` must still wait.
    pub fn holds(&self, frame: u64, now: Instant) -> bool {
        if !self.active {
            return false;
        }
        if self
            .last_heard
            .is_some_and(|t| now.duration_since(t) > SILENT_LEAVES)
        {
            return false;
        }
        self.done.is_none_or(|d| d < frame)
    }
}

/// One follower: its state and what the gate spent on it.
#[derive(Debug, Clone, Default)]
pub struct Follower {
    pub state: GateState,
    /// Ticks that went ahead because this follower did not finish in time.
    pub missed: u64,
}

/// The gate's socket and its followers, by the address their datagrams come from.
pub struct TickGate {
    sock: UdpSocket,
    followers: BTreeMap<SocketAddr, Follower>,
    timeout: Duration,
    /// Ticks that went ahead because a follower did not finish in time.
    pub missed: u64,
    /// Ticks that waited, and how long in all.
    pub waits: u64,
    pub waited: Duration,
    /// Datagrams dropped whole: over a bound, or not parsing (092 K12).
    pub malformed: u64,
    /// Each entity's latest command (the highest frame it answered; a late datagram never
    /// replaces a newer command).
    commands: BTreeMap<String, Commanded>,
}

impl TickGate {
    /// Bind the gate's address (`host:port`).
    pub fn bind(addr: &str, timeout: Duration) -> std::io::Result<Self> {
        let sock = UdpSocket::bind(addr)?;
        sock.set_nonblocking(true)?;
        Ok(Self {
            sock,
            followers: BTreeMap::new(),
            timeout,
            missed: 0,
            waits: 0,
            waited: Duration::ZERO,
            malformed: 0,
            commands: BTreeMap::new(),
        })
    }

    /// The followers known so far, by address.
    pub fn followers(&self) -> &BTreeMap<SocketAddr, Follower> {
        &self.followers
    }

    fn apply(&mut self, from: SocketAddr, msg: GateMsg, now: Instant) {
        let f = self.followers.entry(from).or_default();
        if msg != GateMsg::Bye && !f.state.active {
            tracing::info!("tick gate: follower {from} joined; every tick waits for it");
        }
        if msg == GateMsg::Bye && f.state.active {
            tracing::info!(
                "tick gate: follower {from} left ({} tick(s) went ahead of it)",
                f.missed
            );
        }
        f.state.apply(msg, now);
    }

    /// Each entity's latest command.
    pub fn commands(&self) -> &BTreeMap<String, Commanded> {
        &self.commands
    }

    /// One datagram: its gate message, then its commands.
    fn take(&mut self, bytes: &[u8], from: SocketAddr) {
        match Datagram::parse(bytes) {
            Ok(d) => {
                if let Some(m) = d.msg {
                    self.apply(from, m, Instant::now());
                }
                for command in d.commands {
                    let newer = self
                        .commands
                        .get(&command.entity)
                        .is_none_or(|c| c.frame <= d.frame);
                    if newer {
                        self.commands.insert(
                            command.entity.clone(),
                            Commanded {
                                frame: d.frame,
                                command,
                            },
                        );
                    }
                }
            }
            Err(why) => {
                self.malformed += 1;
                if self.malformed.is_power_of_two() {
                    tracing::warn!(
                        "tick gate: dropped a datagram from {from}: {why} ({} so far)",
                        self.malformed
                    );
                }
            }
        }
    }

    /// Read every datagram waiting, without blocking (a tick takes the commands that arrived
    /// by then, free-running too).
    pub fn drain(&mut self) {
        // One byte over the limit, so an oversized datagram is seen as such, not truncated.
        let mut buf = [0u8; MAX_DATAGRAM + 1];
        while let Ok((n, from)) = self.sock.recv_from(&mut buf) {
            self.take(&buf[..n], from);
        }
    }

    /// Whether any follower holds the tick after `frame`.
    fn holds(&self, frame: u64, now: Instant) -> bool {
        self.followers.values().any(|f| f.state.holds(frame, now))
    }

    /// Before the tick that follows `frame` (the frame scensim ticked last): wait until every
    /// follower finished it, at most the gate's timeout.
    pub fn wait_for(&mut self, frame: u64) {
        self.drain();
        let start = Instant::now();
        if !self.holds(frame, start) {
            return;
        }
        self.waits += 1;
        // Blocking reads with a short timeout, so a datagram ends the wait at once.
        let _ = self.sock.set_nonblocking(false);
        let _ = self.sock.set_read_timeout(Some(Duration::from_millis(2)));
        while self.holds(frame, Instant::now()) {
            if start.elapsed() >= self.timeout {
                self.missed += 1;
                let now = Instant::now();
                for (addr, f) in self.followers.iter_mut() {
                    if f.state.holds(frame, now) {
                        f.missed += 1;
                        if f.missed.is_power_of_two() {
                            tracing::warn!(
                                "tick gate: follower {addr} did not finish frame {frame} within \
                                 {} ms; ticking on ({} miss(es) so far)",
                                self.timeout.as_millis(),
                                f.missed
                            );
                        }
                    }
                }
                break;
            }
            let mut buf = [0u8; MAX_DATAGRAM + 1];
            if let Ok((n, from)) = self.sock.recv_from(&mut buf) {
                self.take(&buf[..n], from);
            }
        }
        let _ = self.sock.set_nonblocking(true);
        self.waited += start.elapsed();
    }
}

/// The gate as a step uses it: [`StepGate::before_step`] waits for the frame the last step
/// ended on, [`StepGate::after_step`] records it. Ticks between steps (a spawn's settling tick)
/// change neither.
pub struct StepGate {
    pub gate: TickGate,
    last_step_frame: Option<u64>,
}

impl StepGate {
    pub fn new(gate: TickGate) -> Self {
        Self {
            gate,
            last_step_frame: None,
        }
    }

    /// Before a step's first tick: every follower finished the frame the previous step ended on.
    pub fn before_step(&mut self) {
        if let Some(frame) = self.last_step_frame {
            self.gate.wait_for(frame);
        }
    }

    /// After a step's last tick: `frame` is the frame it ended on (None: unknown).
    pub fn after_step(&mut self, frame: Option<u64>) {
        self.last_step_frame = frame;
    }

    /// A new world: no step has ended in it.
    pub fn reset(&mut self) {
        self.last_step_frame = None;
    }

    #[cfg(test)]
    pub fn last_step_frame(&self) -> Option<u64> {
        self.last_step_frame
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn messages_parse() {
        assert_eq!(GateMsg::parse("HELLO"), Some(GateMsg::Hello));
        assert_eq!(GateMsg::parse("DONE 42"), Some(GateMsg::Done(42)));
        assert_eq!(GateMsg::parse("BYE\n"), Some(GateMsg::Bye));
        assert_eq!(GateMsg::parse("DONE x"), None);
        assert_eq!(GateMsg::parse("NOPE"), None);
    }

    #[test]
    fn commands_ride_done_and_cmd() {
        let d = Datagram::parse(b"DONE 42 walker 1.25 -0.5 2").unwrap();
        assert_eq!(d.msg, Some(GateMsg::Done(42)));
        assert_eq!(d.frame, 42);
        assert_eq!(
            d.commands,
            vec![Command {
                entity: "walker".into(),
                speed: 1.25,
                heading: -0.5,
                accel_limit: 2.0
            }]
        );
        let d = Datagram::parse(b"CMD 7 a 0 0 1 b 1e-3 3.141592653589793 0.5\n").unwrap();
        assert_eq!(d.msg, None, "CMD joins nothing");
        assert_eq!(d.commands.len(), 2);
        assert_eq!(d.commands[1].heading, std::f64::consts::PI);
        assert_eq!(Datagram::parse(b"DONE 3").unwrap().commands, vec![]);
        assert_eq!(Datagram::parse(b"HELLO").unwrap().msg, Some(GateMsg::Hello));
    }

    #[test]
    fn a_datagram_out_of_bounds_or_malformed_is_refused_whole() {
        for bad in [
            &b"DONE 1 walker 1 0"[..], // three fields
            b"DONE 1 walker -1 0 1",   // negative speed
            b"DONE 1 walker 1 0 0",    // no acceleration
            b"DONE 1 walker NaN 0 1",  // not finite
            b"DONE 1 walker 1 inf 1",
            b"DONE 1 w 1 0 1 w 2 0 1", // twice in one frame
            b"CMD walker 1 0 1",       // no frame
            b"HELLO walker",
            b"STEP 1",
            b"",
            b"\xff\xfe",
        ] {
            assert!(
                Datagram::parse(bad).is_err(),
                "{:?}",
                String::from_utf8_lossy(bad)
            );
        }
        let long = format!("DONE 1 {} 1 0 1", "n".repeat(MAX_NAME + 1));
        assert!(Datagram::parse(long.as_bytes()).is_err());
        let name = "n".repeat(MAX_NAME);
        assert!(Datagram::parse(format!("DONE 1 {name} 1 0 1").as_bytes()).is_ok());
    }

    #[test]
    fn ten_commands_at_the_bounds_fit_one_datagram_and_eleven_do_not() {
        // The spec's bound (092 K12, D4): names of 64 bytes, numbers of 24 characters (the
        // longest shortest round-trip form of an f64), the frame at u64's longest.
        // (24 with a sign: the heading; a speed and a limit are positive, 23 at most)
        let heading = "-1.2345678901234567e-300";
        let positive = "1.2345678901234567e+300";
        assert_eq!((heading.len(), positive.len()), (24, 23));
        let datagram = |n: usize| {
            let mut s = format!("DONE {}", u64::MAX);
            for i in 0..n {
                s += &format!(" {i:0>64} {positive} {heading} {positive}");
            }
            s
        };
        let ten = datagram(10);
        assert!(ten.len() <= MAX_DATAGRAM, "{}", ten.len());
        assert_eq!(Datagram::parse(ten.as_bytes()).unwrap().commands.len(), 10);
        assert!(datagram(11).len() > MAX_DATAGRAM);
    }

    #[test]
    fn the_gate_keeps_each_entitys_newest_command_and_counts_what_it_drops() {
        let mut g = TickGate::bind("127.0.0.1:0", Duration::from_millis(300)).unwrap();
        let addr = g.sock.local_addr().unwrap();
        let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
        peer.send_to(b"CMD 5 walker 1 0 1", addr).unwrap();
        peer.send_to(b"CMD 4 walker 9 0 1", addr).unwrap(); // late: never replaces frame 5
        peer.send_to(b"CMD 6 walker -1 0 1", addr).unwrap(); // refused whole
        peer.send_to(&vec![b'x'; MAX_DATAGRAM + 10], addr).unwrap();
        std::thread::sleep(Duration::from_millis(20));
        g.drain();
        let c = &g.commands()["walker"];
        assert_eq!((c.frame, c.command.speed), (5, 1.0));
        assert_eq!(g.malformed, 2);
        // CMD joined nothing: no tick waits for this peer.
        assert!(!g.holds(5, Instant::now()));
        // DONE carries commands and finishes the frame.
        peer.send_to(b"HELLO", addr).unwrap();
        peer.send_to(b"DONE 6 walker 1.5 0.25 2", addr).unwrap();
        let start = Instant::now();
        g.wait_for(6);
        assert!(start.elapsed() < Duration::from_millis(250));
        assert_eq!(g.commands()["walker"].frame, 6);
        assert_eq!(g.commands()["walker"].command.heading, 0.25);
    }

    #[test]
    fn a_tick_waits_only_for_an_attached_follower_behind_the_frame() {
        let t0 = Instant::now();
        let mut s = GateState::default();
        assert!(!s.holds(10, t0), "no follower: no wait");
        s.apply(GateMsg::Hello, t0);
        assert!(s.holds(10, t0), "joined, nothing finished yet");
        s.apply(GateMsg::Done(9), t0);
        assert!(s.holds(10, t0));
        s.apply(GateMsg::Done(10), t0);
        assert!(!s.holds(10, t0));
        s.apply(GateMsg::Done(8), t0);
        assert_eq!(
            s.done,
            Some(10),
            "a late datagram never moves the frame back"
        );
        s.apply(GateMsg::Bye, t0);
        assert!(!s.holds(11, t0));
    }

    #[test]
    fn a_silent_follower_is_taken_as_gone() {
        let t0 = Instant::now();
        let mut s = GateState::default();
        s.apply(GateMsg::Done(3), t0);
        assert!(s.holds(4, t0));
        assert!(!s.holds(4, t0 + SILENT_LEAVES + Duration::from_millis(1)));
    }

    #[test]
    fn the_gate_waits_until_the_frame_is_done_and_not_longer() {
        let mut g = TickGate::bind("127.0.0.1:0", Duration::from_millis(500)).unwrap();
        let addr = g.sock.local_addr().unwrap();
        let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
        peer.send_to(b"HELLO", addr).unwrap();
        let t = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            peer.send_to(b"DONE 7", addr).unwrap();
        });
        let start = Instant::now();
        g.wait_for(7);
        let took = start.elapsed();
        t.join().unwrap();
        assert!(
            took >= Duration::from_millis(40) && took < Duration::from_millis(400),
            "{took:?}"
        );
        assert_eq!(g.missed, 0);
        // Nothing for frame 8: the wait ends at the timeout and is counted.
        g.wait_for(8);
        assert_eq!(g.missed, 1);
    }

    #[test]
    fn a_step_waits_for_the_frame_the_last_step_ended_on_not_for_ticks_between() {
        // 092 K4: the follower finishes the frames the truth stream reports, one per step. A
        // spawn's settling ticks (frames 11 to 14 here) and a step's substeps carry no truth
        // frame, so waiting for them cost a timeout each.
        let gate = TickGate::bind("127.0.0.1:0", Duration::from_millis(300)).unwrap();
        let addr = gate.sock.local_addr().unwrap();
        let mut g = StepGate::new(gate);
        let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
        peer.send_to(b"HELLO", addr).unwrap();
        g.before_step(); // the first step: nothing ended yet, no wait
        g.after_step(Some(10)); // its last tick was frame 10, which the truth stream reports
        peer.send_to(b"DONE 10", addr).unwrap();
        // settling ticks after spawns: frames 11 to 14, neither recorded nor waited for
        let start = Instant::now();
        g.before_step();
        assert!(
            start.elapsed() < Duration::from_millis(200),
            "{:?}",
            start.elapsed()
        );
        assert_eq!(g.gate.missed, 0);
        // a step with 3 substeps ends on 17; the follower has not finished it: one miss, once
        g.after_step(Some(17));
        g.before_step();
        assert_eq!(g.gate.missed, 1);
        g.reset();
        g.before_step(); // a new world: nothing to wait for
        assert_eq!(g.gate.missed, 1);
        assert_eq!(g.last_step_frame(), None);
    }

    #[test]
    fn a_tick_waits_for_every_follower() {
        let mut g = TickGate::bind("127.0.0.1:0", Duration::from_millis(500)).unwrap();
        let addr = g.sock.local_addr().unwrap();
        let fast = UdpSocket::bind("127.0.0.1:0").unwrap();
        let slow = UdpSocket::bind("127.0.0.1:0").unwrap();
        fast.send_to(b"HELLO", addr).unwrap();
        slow.send_to(b"HELLO", addr).unwrap();
        fast.send_to(b"DONE 5", addr).unwrap();
        let t = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(80));
            slow.send_to(b"DONE 5", addr).unwrap();
            slow
        });
        let start = Instant::now();
        g.wait_for(5);
        let took = start.elapsed();
        let slow = t.join().unwrap();
        assert!(
            took >= Duration::from_millis(70) && took < Duration::from_millis(400),
            "the tick waited for the slower follower: {took:?}"
        );
        assert_eq!(g.followers().len(), 2);
        // Only the slow follower is behind on frame 6: the miss is counted against it.
        fast.send_to(b"DONE 6", addr).unwrap();
        g.wait_for(6);
        assert_eq!(g.missed, 1);
        let behind = g.followers()[&slow.local_addr().unwrap()].missed;
        let ahead = g.followers()[&fast.local_addr().unwrap()].missed;
        assert_eq!((behind, ahead), (1, 0));
        // One leaves: the other alone decides.
        slow.send_to(b"BYE", addr).unwrap();
        fast.send_to(b"DONE 7", addr).unwrap();
        let start = Instant::now();
        g.wait_for(7);
        assert!(start.elapsed() < Duration::from_millis(100));
        assert_eq!(g.missed, 1);
    }
}
