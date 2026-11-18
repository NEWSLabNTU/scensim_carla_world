//! Town names: the scenario's against the CARLA server's (roadmap 083).
//!
//! Scenarios, Autoware maps and the signal tables name the classic towns as 0.9.x does:
//! `Town01`. CARLA 1.0 ships them only as layered maps, loaded by their `_Opt` name
//! (`Town01_Opt`, in the AdditionalMaps pack), and lists no plain `Town01`. So on a 1.0
//! server a requested town that the server does not list is loaded by its `_Opt` name when
//! the server has that, and the server's `_Opt` name reads back as the scenario's town.
//! 0.9.x servers list both forms and are left exactly as before: their names pass through.
//!
//! The version is this binary's (`version::BUILT_FOR`): the connect check refuses a server
//! of any other release, so it is the server's too.

/// The suffix CARLA gives a town's layered version.
const LAYERED: &str = "_Opt";

/// Whether CARLA `version` ships the classic towns only as layered (`_Opt`) maps: 1.0 on.
pub fn layered_only(version: (u32, u32, u32)) -> bool {
    version >= (1, 0, 0)
}

/// The final component of a CARLA map path: `"Carla/Maps/Town01_Opt"` → `"Town01_Opt"`.
pub fn short_name(map: &str) -> &str {
    map.rsplit('/').next().unwrap_or(map)
}

/// The scenario's name for the town CARLA reports as `map` (a path or a short name).
/// On a layered-only server `Town01_Opt` reads as `Town01`; otherwise the short name as is.
pub fn logical(map: &str, version: (u32, u32, u32)) -> String {
    let short = short_name(map);
    if layered_only(version) {
        if let Some(base) = short.strip_suffix(LAYERED) {
            if !base.is_empty() {
                return base.to_string();
            }
        }
    }
    short.to_string()
}

/// The name to load `town` by on a server listing `available` (paths or short names), or
/// `None` when the server has no such town. A name the server lists is used as given; on a
/// layered-only server a town it lists only as `<town>_Opt` is loaded by that name.
pub fn on_server(town: &str, available: &[String], version: (u32, u32, u32)) -> Option<String> {
    let listed = |name: &str| available.iter().any(|m| short_name(m) == name);
    if listed(town) {
        return Some(town.to_string());
    }
    if layered_only(version) {
        let layered = format!("{town}{LAYERED}");
        if listed(&layered) {
            return Some(layered);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    const V0916: (u32, u32, u32) = (0, 9, 16);
    const V10: (u32, u32, u32) = (1, 0, 0);

    /// `get_available_maps()` on the packaged 1.0 with AdditionalMaps (083 probe, 139).
    fn maps_1_0() -> Vec<String> {
        [
            "Eixample",
            "EmptyMap",
            "Mine_01",
            "Town01_Opt",
            "Town02_Opt",
            "Town03_Opt",
            "Town04_Opt",
            "Town05_Opt",
            "Town06_Opt",
            "Town07_Opt",
            "Town10HD_Opt",
            "Town11",
            "Town12",
            "Town13",
            "Town15",
            "Town_C",
        ]
        .iter()
        .map(|m| format!("/Game/Carla/Maps/{m}"))
        .collect()
    }

    fn maps_0916() -> Vec<String> {
        [
            "Town01",
            "Town01_Opt",
            "Town02",
            "Town02_Opt",
            "Town10HD",
            "Town10HD_Opt",
        ]
        .iter()
        .map(|m| format!("/Game/Carla/Maps/{m}"))
        .collect()
    }

    #[test]
    fn a_classic_town_loads_by_its_layered_name_on_1_0() {
        assert_eq!(
            on_server("Town01", &maps_1_0(), V10).as_deref(),
            Some("Town01_Opt")
        );
        assert_eq!(
            on_server("Town10HD", &maps_1_0(), V10).as_deref(),
            Some("Town10HD_Opt")
        );
        // asked for by its layered name, or a town that has no other form
        assert_eq!(
            on_server("Town01_Opt", &maps_1_0(), V10).as_deref(),
            Some("Town01_Opt")
        );
        assert_eq!(
            on_server("Town_C", &maps_1_0(), V10).as_deref(),
            Some("Town_C")
        );
        assert_eq!(on_server("Town99", &maps_1_0(), V10), None);
    }

    #[test]
    fn a_0916_server_is_unchanged() {
        assert_eq!(
            on_server("Town01", &maps_0916(), V0916).as_deref(),
            Some("Town01")
        );
        assert_eq!(
            on_server("Town01_Opt", &maps_0916(), V0916).as_deref(),
            Some("Town01_Opt")
        );
        // no fallback below 1.0: a town the server lacks stays missing
        let only_opt = vec!["/Game/Carla/Maps/Town03_Opt".to_string()];
        assert_eq!(on_server("Town03", &only_opt, V0916), None);
    }

    #[test]
    fn the_layered_name_reads_back_as_the_scenario_town_on_1_0() {
        assert_eq!(logical("Carla/Maps/Town01_Opt", V10), "Town01");
        assert_eq!(logical("Town10HD_Opt", V10), "Town10HD");
        assert_eq!(logical("Carla/Maps/Town_C", V10), "Town_C");
        assert_eq!(logical("_Opt", V10), "_Opt");
        // 0.9.16 keeps the name it reports: Town01 and Town01_Opt are different worlds there
        assert_eq!(logical("Carla/Maps/Town01", V0916), "Town01");
        assert_eq!(logical("Carla/Maps/Town01_Opt", V0916), "Town01_Opt");
    }

    #[test]
    fn a_loaded_layered_town_counts_as_the_requested_one() {
        // what load_world compares: the logical names of both sides
        let loaded = logical("Carla/Maps/Town01_Opt", V10);
        assert_eq!(loaded, logical("Town01", V10));
    }
}
