from __future__ import annotations

import json
from pathlib import Path

import pytest

from fidelity_bench import mesh_key, role_of
from push_bench import choose_probes

RESULTS = Path(__file__).parent / "results"
LINK = "/World/Stem/Vegetative/Branch_g1/Branch_g1_Link_01_g1"


def test_mesh_key_survives_link_merging():
    plain = f"/World/Stem/Vegetative/Branch_g1/Branch_g1_Link_02_g2/OrganicVisual_02"
    merged = f"{LINK}/Merged_Branch_g1_Link_02_g2/OrganicVisual_02"
    assert mesh_key(plain) == mesh_key(merged) == "Branch_g1_Link_02_g2/OrganicVisual_02"


def test_mesh_key_keeps_leaflets_apart():
    rachis = "/World/Stem/Vegetative/Leaf_g5_rachis/Leaf_g5_rachis_Link_01_g6/RigidLeafVisuals"
    left = mesh_key(f"{rachis}/Leaflet_g5_left01/OrganicVisual_01")
    right = mesh_key(f"{rachis}/Leaflet_g5_right01/OrganicVisual_01")
    assert left != right
    assert role_of(left) == "leaflet"


@pytest.mark.parametrize(
    ("key", "role"),
    [
        ("Truss_g7_truss01_pedicel_01_Link_01_g8/OrganicVisual_01", "pedicel"),
        ("Truss_g7_truss01_rachis_Link_01_g7/OrganicVisual_01", "rachis"),
        ("Leaf_g5_petiole_Link_01_g5/OrganicVisual_01", "petiole"),
        ("Branch_g1_Link_02_g2/OrganicVisual_02", "branch"),
        ("trunk_Link_01_g0/OrganicVisual_01", "stem"),
    ],
)
def test_role_of(key, role):
    assert role_of(key) == role


def test_choose_probes_takes_the_farthest_organ_of_each_kind():
    records = {
        "Branch_g1_Link_01_g1/OrganicVisual_01": {"rest": [0.1, 0.0, 0.5]},
        "Branch_g1_Link_04_g4/OrganicVisual_04": {"rest": [0.4, 0.0, 0.6]},
        "Truss_g7_truss01_rachis_Link_02_g9/OrganicVisual_01": {"rest": [0.0, 0.2, 0.4]},
        "Leaflet_g5_left01/OrganicVisual_01": {"rest": [0.0, -0.3, 0.7]},
        "trunk_Link_01_g0/OrganicVisual_01": {"rest": [0.0, 0.0, 0.1]},
    }
    probes = choose_probes(records, (0.0, 0.0))
    assert probes == {
        "lateral_tip": "Branch_g1_Link_04_g4/OrganicVisual_04",
        "truss_tip": "Truss_g7_truss01_rachis_Link_02_g9/OrganicVisual_01",
        "leaf_blade": "Leaflet_g5_left01/OrganicVisual_01",
    }


def test_saved_results_give_the_published_cost_model():
    pytest.importorskip("matplotlib")
    from analyze import fit, load

    rows = load(sorted((RESULTS / "interactive").glob("*.json")))["rows"]
    cpu = fit([r for r in rows if r["device"] == "cpu"])
    assert cpu["ms_per_dof"] * 1e3 == pytest.approx(2.8, abs=0.05)
    assert cpu["ms_per_body"] * 1e3 == pytest.approx(41.2, abs=0.5)
    assert cpu["r2"] == pytest.approx(0.995, abs=0.001)


def test_saved_fidelity_results_are_the_published_ones():
    rows = {row["variant"]: row for row in json.loads((RESULTS / "fidelity_day160.json").read_text())}
    assert rows["b40_load"]["mean_mm"] == pytest.approx(3.0, abs=0.05)
    assert rows["b40_series"]["max_mm"] == pytest.approx(96.1, abs=0.05)
