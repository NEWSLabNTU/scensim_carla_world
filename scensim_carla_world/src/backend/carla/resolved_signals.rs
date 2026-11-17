//! The resolved traffic light table: what acb_bridge needs to publish CARLA's signal state to
//! Autoware (roadmap 015, "The lanelet <-> CARLA light mapping is produced by scensim and consumed
//! by acb through a file").
//!
//! This bridge already resolves which CARLA light is which Lanelet2 signal (the explicit
//! `config/traffic_lights_<town>.yaml` merged with the position match). At Initialize it
//! checks that result against the map artifact `<map dir>/traffic_lights.yaml`:
//!
//! ```yaml
//! town: Town01
//! lanelet2_map: /.../Town01/lanelet2_map.osm
//! signals:                      # one entry per mapped Lanelet2 traffic light way
//!   - way_id: 43763             # what SSv2 commands (TrafficSignal.id)
//!     regulatory_element_ids: [43856]   # what Autoware keys TrafficLightGroup on
//!     opendrive_id: "12"        # World::traffic_light_from_open_drive
//!     carla_position: [92.1, -55.3, 0.0]   # CARLA frame, metres (the actor origin)
//! unmapped:
//!   lanelet_way_ids: []         # Lanelet2 lights with no CARLA light
//!   carla_opendrive_ids: []     # CARLA lights the Lanelet2 map does not reference
//! ```
//!
//! Since roadmap 017 the table is a **map artifact**, generated once per map with
//! `scensim_carla_world --generate-signal-table <map dir>` into
//! `<map dir>/traffic_lights.yaml` (roadmap 046: one table every simulator with signals
//! reads; `carla/traffic_lights.yaml` before, still read and kept as a link to it).
//! At Initialize scensim resolves the mapping again and **checks** it against that file instead
//! of writing into the map directory: the map dir is the user's, read-only at run time.
//! A difference fails Initialize, because acb would publish signals for the wrong lights.
//!
//! Since roadmap 076 (A9) nobody has to run the generator. The table records the SHA-256 of
//! the Lanelet2 file it was resolved from (`lanelet2_sha256`), and at Initialize, after the
//! world load, the server writes it itself when it is missing or stale (that hash differs),
//! into the stack's state directory when one is given (`state_dir`:
//! `<state_dir>/<map>/traffic_lights.yaml`, with `<state_dir>/<map>/carla/traffic_lights.yaml`
//! a link to it, which acb reads first), else beside the map as the generator does. A table
//! that differs from what was resolved, with the same map, is rewritten in the state
//! directory (the stack's own) and still fails Initialize in a map directory (the user's).

use std::path::{Path, PathBuf};

use eyre::{Result, WrapErr};
use serde::Serialize;

use super::traffic_light_mapper::{CarlaSignal, SignalMap};
use crate::lanelet_map::TrafficLightElement;

/// Where the table lives, relative to the map directory.
pub const TABLE_SUBPATH: &str = crate::map_resolver::SIGNAL_TABLE_SUBPATH;

#[derive(Debug, Serialize, PartialEq)]
pub struct ResolvedSignal {
    pub way_id: i32,
    pub regulatory_element_ids: Vec<i64>,
    pub opendrive_id: String,
    /// `None` when the explicit mapping names a sign CARLA does not have.
    pub carla_position: Option<[f64; 3]>,
}

#[derive(Debug, Default, Serialize, PartialEq)]
pub struct Unmapped {
    pub lanelet_way_ids: Vec<i32>,
    pub carla_opendrive_ids: Vec<String>,
}

#[derive(Debug, Serialize, PartialEq)]
pub struct ResolvedTable {
    pub town: String,
    pub lanelet2_map: String,
    /// SHA-256 of the Lanelet2 file, hex: what makes a table stale. `None` (not written) when
    /// the file could not be read.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lanelet2_sha256: Option<String>,
    pub signals: Vec<ResolvedSignal>,
    pub unmapped: Unmapped,
}

impl ResolvedTable {
    /// Join the mapping with the Lanelet2 lights (for their regulatory elements) and CARLA's
    /// lights (for their positions). Every mapped way is listed, even one whose regulatory
    /// element or CARLA light is unknown, so the gap is visible in the file.
    pub fn build(
        town: &str,
        lanelet2_map: &Path,
        map: &SignalMap,
        lanelet: &[TrafficLightElement],
        carla: &[CarlaSignal],
    ) -> Self {
        let entries = map.entries();
        let signals = entries
            .iter()
            .map(|&(way_id, opendrive_id)| ResolvedSignal {
                way_id,
                regulatory_element_ids: lanelet
                    .iter()
                    .find(|l| l.way_id == way_id)
                    .map(|l| l.regulatory_element_ids.clone())
                    .unwrap_or_default(),
                opendrive_id: opendrive_id.to_string(),
                carla_position: carla
                    .iter()
                    .find(|c| c.opendrive_id == opendrive_id)
                    .map(|c| [c.x, c.y, c.z]),
            })
            .collect();

        let mut lanelet_way_ids: Vec<i32> = lanelet
            .iter()
            .map(|l| l.way_id)
            .filter(|id| !entries.iter().any(|(w, _)| w == id))
            .collect();
        lanelet_way_ids.sort_unstable();
        let mut carla_opendrive_ids: Vec<String> = carla
            .iter()
            .map(|c| c.opendrive_id.clone())
            .filter(|id| !entries.iter().any(|(_, o)| o == id))
            .collect();
        carla_opendrive_ids.sort();

        Self {
            town: town.to_string(),
            lanelet2_map: lanelet2_map.display().to_string(),
            lanelet2_sha256: lanelet2_sha256(lanelet2_map),
            signals,
            unmapped: Unmapped {
                lanelet_way_ids,
                carla_opendrive_ids,
            },
        }
    }

    /// Distinct regulatory elements with at least one mapped light, for the log line.
    pub fn regulatory_element_count(&self) -> usize {
        let mut ids: Vec<i64> = self
            .signals
            .iter()
            .flat_map(|s| s.regulatory_element_ids.iter().copied())
            .collect();
        ids.sort_unstable();
        ids.dedup();
        ids.len()
    }

    pub fn to_yaml(&self) -> Result<String> {
        let body = serde_yaml::to_string(self).wrap_err("serialize the resolved table")?;
        Ok(format!(
            "# Generated by scensim_carla_world (at an Initialize that found it missing or stale, or\n\
             # --generate-signal-table); read by acb_bridge (traffic_light_state_path, then\n\
             # traffic_light_map_path) and checked by scensim at every Initialize. Do not edit:\n\
             # correct config/traffic_lights_<town>.yaml and regenerate. Roadmap 015, 017, 076.\n{body}"
        ))
    }
}

pub use crate::map_resolver::map_dir_of;

/// SHA-256 of the file at `path`, hex; `None` when it cannot be read.
pub fn lanelet2_sha256(path: &Path) -> Option<String> {
    use sha2::{Digest, Sha256};
    let bytes = std::fs::read(path).ok()?;
    Some(format!("{:x}", Sha256::digest(&bytes)))
}

/// The table to check: `<map dir>/traffic_lights.yaml`, or the older
/// `<map dir>/carla/traffic_lights.yaml` when only that exists.
pub fn table_path(map_dir: &Path) -> PathBuf {
    crate::map_resolver::signal_table(map_dir)
}

/// The directory a table for the map at `map_dir` is written to: `<state_dir>/<map>` (the
/// map directory's name) when the stack gives a state directory, else the map directory.
pub fn table_dir(map_dir: &Path, state_dir: Option<&Path>) -> PathBuf {
    match state_dir {
        Some(state) => {
            let name = map_dir
                .file_name()
                .map(|n| n.to_os_string())
                .unwrap_or_else(|| "map".into());
            state.join(name)
        }
        None => map_dir.to_path_buf(),
    }
}

/// The table to check, in the order a vehicle bridge reads them: the state directory's
/// when it has one, else the map directory's (`table_path`), else where it would be written.
pub fn find_table(map_dir: &Path, state_dir: Option<&Path>) -> PathBuf {
    let dest = table_path(&table_dir(map_dir, state_dir));
    if dest.exists() {
        return dest;
    }
    let beside = table_path(map_dir);
    if beside.exists() {
        beside
    } else {
        dest
    }
}

/// What Initialize does with the table it found.
#[derive(Debug, PartialEq)]
pub enum Decision {
    /// It matches: nothing to write.
    Keep,
    /// Write it (into `table_dir`), saying why.
    Write(String),
    /// Refuse: a map directory's table that disagrees with CARLA, for the same map.
    Fail(String),
}

/// Decide from the check: a missing or stale table is written; one that differs for the
/// same map is rewritten when the stack has a state directory (its own files) and refused
/// otherwise (the map directory is the user's: correct the mapping and regenerate).
pub fn decide(check: &Check, have_state_dir: bool) -> Decision {
    match check {
        Check::Matches => Decision::Keep,
        Check::Missing => Decision::Write("missing".into()),
        Check::Stale => Decision::Write("stale: its Lanelet2 map changed".into()),
        Check::Differs(why) if have_state_dir => {
            Decision::Write(format!("{why} from the resolved one"))
        }
        Check::Differs(why) => Decision::Fail(why.clone()),
    }
}

/// Write the table into `map_dir` atomically (acb re-reads it on change and must never see
/// half of it), and point the older `carla/traffic_lights.yaml` at it, for a vehicle bridge
/// still configured with that path. Only the generator writes; Initialize checks.
pub fn write_table(map_dir: &Path, contents: &str) -> Result<PathBuf> {
    let path = map_dir.join(TABLE_SUBPATH);
    let written = write_atomically(&path, contents)?;
    let old = map_dir.join(crate::map_resolver::OLD_SIGNAL_TABLE_SUBPATH);
    if let Some(dir) = old.parent() {
        std::fs::create_dir_all(dir).wrap_err_with(|| format!("create {}", dir.display()))?;
    }
    let _ = std::fs::remove_file(&old);
    std::os::unix::fs::symlink(Path::new("..").join(TABLE_SUBPATH), &old)
        .wrap_err_with(|| format!("link {}", old.display()))?;
    Ok(written)
}

fn write_atomically(path: &Path, contents: &str) -> Result<PathBuf> {
    let path = path.to_path_buf();
    let dir = path.parent().expect("TABLE_SUBPATH has a parent");
    std::fs::create_dir_all(dir).wrap_err_with(|| format!("create {}", dir.display()))?;
    let tmp = dir.join(format!(".traffic_lights.yaml.{}.tmp", std::process::id()));
    std::fs::write(&tmp, contents).wrap_err_with(|| format!("write {}", tmp.display()))?;
    if let Err(e) = std::fs::rename(&tmp, &path) {
        let _ = std::fs::remove_file(&tmp);
        return Err(e).wrap_err_with(|| format!("rename onto {}", path.display()));
    }
    Ok(path)
}

/// How the table on disk compares with the one just resolved.
#[derive(Debug, PartialEq)]
pub enum Check {
    Matches,
    Missing,
    /// Resolved from another Lanelet2 file: both record a `lanelet2_sha256` and they differ.
    Stale,
    /// The first difference, for the error message.
    Differs(String),
}

/// Compare the file at `path` with `table`, ignoring `lanelet2_map` (the same map is
/// spelled differently wherever it is mounted) and comments. A table whose recorded
/// `lanelet2_sha256` is not the map's is `Stale`; one without (written before roadmap 076)
/// is compared by content alone.
pub fn check_table(path: &Path, table: &ResolvedTable) -> Result<Check> {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Check::Missing),
        Err(e) => return Err(e).wrap_err_with(|| format!("read {}", path.display())),
    };
    let mut on_disk: serde_yaml::Value =
        serde_yaml::from_str(&text).wrap_err_with(|| format!("parse {}", path.display()))?;
    let recorded = on_disk
        .get("lanelet2_sha256")
        .and_then(|v| v.as_str())
        .map(str::to_string);
    if let (Some(recorded), Some(current)) = (&recorded, &table.lanelet2_sha256) {
        if recorded != current {
            return Ok(Check::Stale);
        }
    }
    let mut resolved = serde_yaml::to_value(table).wrap_err("serialize the resolved table")?;
    for v in [&mut on_disk, &mut resolved] {
        if let Some(m) = v.as_mapping_mut() {
            m.remove("lanelet2_map");
            m.remove("lanelet2_sha256");
        }
    }
    if on_disk == resolved {
        return Ok(Check::Matches);
    }
    let field = ["town", "signals", "unmapped"]
        .into_iter()
        .find(|k| on_disk.get(k) != resolved.get(k))
        .unwrap_or("structure");
    Ok(Check::Differs(format!("`{field}` differs")))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn light(way_id: i32, reg: &[i64]) -> TrafficLightElement {
        TrafficLightElement {
            way_id,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            regulatory_element_ids: reg.to_vec(),
        }
    }

    fn carla(id: &str, x: f64, y: f64, z: f64) -> CarlaSignal {
        CarlaSignal {
            opendrive_id: id.to_string(),
            x,
            y,
            z,
        }
    }

    fn table() -> ResolvedTable {
        let map = SignalMap::from_pairs(&[("43763", "12"), ("43760", "13"), ("99", "77")]);
        let lanelet = [
            light(43763, &[43856]),
            light(43760, &[43856]),
            light(500, &[501]),
        ];
        let carla_lights = [
            carla("12", 1.0, -2.0, 0.5),
            carla("13", 3.0, -4.0, 0.5),
            carla("40", 0.0, 0.0, 0.0),
        ];
        ResolvedTable::build(
            "Town01",
            Path::new("/m/Town01/lanelet2_map.osm"),
            &map,
            &lanelet,
            &carla_lights,
        )
    }

    #[test]
    fn every_mapped_way_names_its_regulatory_element_and_carla_light() {
        let t = table();
        assert_eq!(
            t.signals,
            vec![
                ResolvedSignal {
                    way_id: 99,
                    regulatory_element_ids: vec![],
                    opendrive_id: "77".into(),
                    carla_position: None,
                },
                ResolvedSignal {
                    way_id: 43760,
                    regulatory_element_ids: vec![43856],
                    opendrive_id: "13".into(),
                    carla_position: Some([3.0, -4.0, 0.5]),
                },
                ResolvedSignal {
                    way_id: 43763,
                    regulatory_element_ids: vec![43856],
                    opendrive_id: "12".into(),
                    carla_position: Some([1.0, -2.0, 0.5]),
                },
            ]
        );
        assert_eq!(t.regulatory_element_count(), 1);
    }

    #[test]
    fn unmapped_lights_are_listed_on_both_sides() {
        let t = table();
        assert_eq!(t.unmapped.lanelet_way_ids, vec![500]);
        assert_eq!(t.unmapped.carla_opendrive_ids, vec!["40".to_string()]);
    }

    #[test]
    fn the_yaml_round_trips_the_fields_acb_reads() {
        let yaml = table().to_yaml().unwrap();
        let v: serde_yaml::Value = serde_yaml::from_str(&yaml).unwrap();
        let s = &v["signals"][2];
        assert_eq!(s["way_id"].as_i64(), Some(43763));
        assert_eq!(s["regulatory_element_ids"][0].as_i64(), Some(43856));
        assert_eq!(s["opendrive_id"].as_str(), Some("12"));
        assert_eq!(s["carla_position"][1].as_f64(), Some(-2.0));
        assert!(v["signals"][0]["carla_position"].is_null());
    }

    #[test]
    fn generated_table_checks_clean_and_detects_a_change() {
        let base = std::env::temp_dir().join(format!("scensim_table_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&base);
        let path = table_path(&base);
        assert_eq!(check_table(&path, &table()).unwrap(), Check::Missing);

        let written = write_table(&base, &table().to_yaml().unwrap()).unwrap();
        assert_eq!(written, base.join("traffic_lights.yaml"));
        assert_eq!(check_table(&path, &table()).unwrap(), Check::Matches);
        // The older path still reads the table (a link to it), for acb's default.
        let old = base.join("carla/traffic_lights.yaml");
        assert_eq!(
            std::fs::read_to_string(&old).unwrap(),
            std::fs::read_to_string(&written).unwrap()
        );
        assert_eq!(table_path(&base), written);

        // The map path is spelled differently where it is mounted; that is not a change.
        let mut moved = table();
        moved.lanelet2_map = "/elsewhere/lanelet2_map.osm".into();
        assert_eq!(check_table(&path, &moved).unwrap(), Check::Matches);

        let mut changed = table();
        changed.signals.pop();
        assert!(
            matches!(check_table(&path, &changed).unwrap(), Check::Differs(d) if d.contains("signals"))
        );
        let _ = std::fs::remove_dir_all(&base);
    }

    fn scratch(name: &str) -> PathBuf {
        let base = std::env::temp_dir().join(format!("scensim_{name}_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&base);
        std::fs::create_dir_all(&base).unwrap();
        base
    }

    /// A table resolved from a real Lanelet2 file at `<base>/Town01/lanelet2_map.osm`.
    fn table_from(map_file: &Path) -> ResolvedTable {
        let map = SignalMap::from_pairs(&[("43763", "12")]);
        ResolvedTable::build(
            "Town01",
            map_file,
            &map,
            &[light(43763, &[43856])],
            &[carla("12", 1.0, -2.0, 0.5)],
        )
    }

    #[test]
    fn the_table_records_its_lanelet2_hash_and_goes_stale_when_the_map_changes() {
        let base = scratch("stale");
        let map_dir = base.join("Town01");
        std::fs::create_dir_all(&map_dir).unwrap();
        let osm = map_dir.join("lanelet2_map.osm");
        std::fs::write(&osm, "<osm version='0.6'/>\n").unwrap();

        let t = table_from(&osm);
        let hash = t.lanelet2_sha256.clone().expect("readable map is hashed");
        assert_eq!(hash.len(), 64);
        let written = write_table(&map_dir, &t.to_yaml().unwrap()).unwrap();
        let v: serde_yaml::Value =
            serde_yaml::from_str(&std::fs::read_to_string(&written).unwrap()).unwrap();
        assert_eq!(v["lanelet2_sha256"].as_str(), Some(hash.as_str()));
        assert_eq!(check_table(&written, &t).unwrap(), Check::Matches);

        // The map is edited: the same lights, but the table is stale.
        std::fs::write(&osm, "<osm version='0.6'><!-- moved --></osm>\n").unwrap();
        let t2 = table_from(&osm);
        assert_ne!(t2.lanelet2_sha256, t.lanelet2_sha256);
        assert_eq!(check_table(&written, &t2).unwrap(), Check::Stale);

        // A table written before the hash (none recorded) is compared by content only.
        let old = written.with_file_name("old.yaml");
        let mut no_hash = table_from(&osm);
        no_hash.lanelet2_sha256 = None;
        std::fs::write(&old, no_hash.to_yaml().unwrap()).unwrap();
        assert!(!std::fs::read_to_string(&old)
            .unwrap()
            .contains("lanelet2_sha256"));
        assert_eq!(check_table(&old, &t2).unwrap(), Check::Matches);

        // An unreadable map records no hash and is never stale on that account.
        let mut unread = table_from(&base.join("nowhere.osm"));
        assert!(unread.lanelet2_sha256.is_none());
        assert_eq!(check_table(&written, &unread).unwrap(), Check::Matches);
        unread.signals.clear();
        assert!(matches!(
            check_table(&written, &unread).unwrap(),
            Check::Differs(_)
        ));
        let _ = std::fs::remove_dir_all(&base);
    }

    #[test]
    fn the_state_dir_holds_the_table_under_the_maps_name() {
        let map_dir = Path::new("/maps/Town01");
        assert_eq!(table_dir(map_dir, None), map_dir);
        assert_eq!(
            table_dir(map_dir, Some(Path::new("/p/state"))),
            Path::new("/p/state/Town01")
        );
        assert_eq!(
            table_dir(Path::new("/"), Some(Path::new("/p/state"))),
            Path::new("/p/state/map")
        );
    }

    #[test]
    fn the_table_is_found_in_the_state_dir_first_then_beside_the_map() {
        let base = scratch("find");
        let map_dir = base.join("maps/Town01");
        let state = base.join("project/state");
        std::fs::create_dir_all(&map_dir).unwrap();
        let in_state = state.join("Town01/traffic_lights.yaml");
        let beside = map_dir.join("traffic_lights.yaml");

        // Nothing anywhere: where it will be written.
        assert_eq!(find_table(&map_dir, Some(&state)), in_state);
        assert_eq!(find_table(&map_dir, None), beside);

        // Only beside the map: that one, with or without a state dir.
        write_table(&map_dir, "town: Town01\n").unwrap();
        assert_eq!(find_table(&map_dir, Some(&state)), beside);
        assert_eq!(find_table(&map_dir, None), beside);

        // Written to the state dir: it wins, and its carla/ link is what acb reads.
        let written = write_table(&table_dir(&map_dir, Some(&state)), "town: Town01\n").unwrap();
        assert_eq!(written, in_state);
        assert_eq!(find_table(&map_dir, Some(&state)), in_state);
        assert_eq!(
            std::fs::read_to_string(state.join("Town01/carla/traffic_lights.yaml")).unwrap(),
            "town: Town01\n"
        );
        // Without a state dir the map dir's is still the one.
        assert_eq!(find_table(&map_dir, None), beside);
        let _ = std::fs::remove_dir_all(&base);
    }

    #[test]
    fn missing_and_stale_tables_are_written_and_a_differing_one_only_in_the_state_dir() {
        for state in [false, true] {
            assert_eq!(decide(&Check::Matches, state), Decision::Keep);
            assert!(matches!(decide(&Check::Missing, state), Decision::Write(_)));
            assert!(matches!(decide(&Check::Stale, state), Decision::Write(_)));
        }
        let differs = Check::Differs("`signals` differs".into());
        assert!(matches!(decide(&differs, true), Decision::Write(w) if w.contains("signals")));
        assert!(matches!(decide(&differs, false), Decision::Fail(w) if w.contains("signals")));
    }

    #[test]
    fn map_dir_of_a_file_or_a_directory() {
        let base = std::env::temp_dir().join(format!("scensim_mapdir_{}", std::process::id()));
        std::fs::create_dir_all(&base).unwrap();
        assert_eq!(map_dir_of(&base), base);
        assert_eq!(map_dir_of(&base.join("lanelet2_map.osm")), base);
        let _ = std::fs::remove_dir_all(&base);
    }
}
