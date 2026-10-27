from exporterV2.core.optimizations.optimizer import BudgetOptimizer
from exporterV2.core.optimizations.techniques.stem_collapse import StemCollapseTechnique


def test_fixed_trunk_is_excluded_from_stem_optimization():
    branches = [
        {
            "id": "trunk",
            "parent": None,
            "n_links": 8,
            "height": 0.10,
            "radius": 0.02,
            "joint_type": "fixed",
        }
    ]
    technique = StemCollapseTechnique(target_segments=3)

    assert not technique.can_apply(branches)
    assert technique.estimate_reduction(branches) == 0
    unchanged, report = technique.apply(branches)
    assert unchanged == branches
    assert report.joints_saved == 0
    assert BudgetOptimizer(max_joints=250).calculate_lower_bound(branches) == 0
