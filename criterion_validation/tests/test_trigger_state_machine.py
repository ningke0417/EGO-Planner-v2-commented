#!/usr/bin/env python3

from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from sweep_split_merge_trigger import case_labels, simulate  # noqa: E402


PARTITION = ((0,), (1,))


def snapshot(x, best_cut, active_cut=None, internal=0.8):
    if active_cut is None:
        active_cut = best_cut
    return {
        "x": x,
        "best_partition": PARTITION,
        "best_features": {
            "mean_cut": best_cut,
            "mean_internal": internal,
        },
        "partition_features": {
            PARTITION: {"mean_cut": active_cut, "mean_internal": internal},
        },
    }


class TriggerStateMachineTest(unittest.TestCase):
    def test_case_labels_use_requested_metric(self):
        import tempfile

        content = (
            "snapshot_id,metric,objective,oracle_translation_distance,"
            "any_uav_in_obstacle,intact_feasible,valid_partition_count\n"
            "1,composite_min,mean_cut,0.5,0,0,0\n"
            "1,real_composite_min,mean_cut,0.5,0,0,1\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rankings.csv"
            path.write_text(content, encoding="utf-8")
            labels = case_labels(path, 0.5, "real_composite_min")

        self.assertEqual(labels[1]["valid_partition_count"], 1)

    def test_transient_low_score_does_not_split(self):
        case = {
            "first_blocked_x": None,
            "snapshots": [
                snapshot(0.0, 0.4),
                snapshot(0.2, 0.8),
                snapshot(0.4, 0.8),
            ],
        }
        result = simulate(case, 0.46, 0.75, 0.5, 0.3)
        self.assertIsNone(result["split_x"])

    def test_sustained_low_score_splits_then_uses_higher_merge_threshold(self):
        case = {
            "first_blocked_x": 0.4,
            "snapshots": [
                snapshot(0.0, 0.4),
                snapshot(0.2, 0.4),
                snapshot(0.4, 0.4),
                snapshot(0.6, 0.4, active_cut=0.7),
                snapshot(0.8, 0.8, active_cut=0.8),
                snapshot(1.0, 0.8, active_cut=0.8),
                snapshot(1.2, 0.8, active_cut=0.8),
            ],
        }
        result = simulate(case, 0.46, 0.75, 0.5, 0.3)
        self.assertEqual(result["split_x"], 0.4)
        self.assertEqual(result["merge_x"], 1.2)
        self.assertTrue(result["detected_before_block"])
        self.assertFalse(result["premature_merge"])


if __name__ == "__main__":
    unittest.main()
