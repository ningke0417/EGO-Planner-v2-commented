#!/usr/bin/env python3
"""Generate raw edge scores for a deterministic single-pillar parameter sweep."""

import argparse
import csv
import json
import math
from pathlib import Path
import sys
from typing import Any, Dict, Iterable, List, Sequence

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from criterion.geometry import (  # noqa: E402
    Pillar,
    Vec2,
    build_star_space,
    forward_window_mask,
    pair_score,
    sampled_overlap,
    sample_grid_memberships,
)


def load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping")
    return data


def inclusive_values(start: float, end: float, step: float) -> Iterable[float]:
    if step <= 0.0 or end < start:
        raise ValueError("invalid inclusive numeric range")
    count = int(math.floor((end - start) / step + 1e-9))
    for index in range(count + 1):
        yield round(start + index * step, 12)
    if start + count * step < end - 1e-9:
        yield end


def validate_sweep(sweep: Dict[str, Any]) -> None:
    radii = [float(value) for value in sweep["pillar_radii"]]
    offsets = [float(value) for value in sweep["pillar_y_offsets"]]
    if not radii or any(value <= 0.0 for value in radii):
        raise ValueError("pillar_radii must contain positive values")
    if not offsets or not all(math.isfinite(value) for value in offsets):
        raise ValueError("pillar_y_offsets must contain finite values")
    centre = sweep["formation_center"]
    list(inclusive_values(
        float(centre["x_start"]), float(centre["x_end"]), float(centre["x_step"])
    ))
    if float(sweep["area_sample_resolution"]) <= 0.0:
        raise ValueError("area_sample_resolution must be positive")
    distances = [float(value) for value in sweep["oracle_translation_distances"]]
    if not distances or any(value <= 0.0 for value in distances):
        raise ValueError("oracle_translation_distances must contain positive values")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--formation", required=True, type=Path)
    parser.add_argument("--base-scenario", required=True, type=Path)
    parser.add_argument("--sweep", required=True, type=Path)
    parser.add_argument("--csv-out", required=True, type=Path)
    parser.add_argument("--summary-out", type=Path)
    args = parser.parse_args()

    formation = load_yaml(args.formation)
    scenario = load_yaml(args.base_scenario)
    sweep = load_yaml(args.sweep)
    validate_sweep(sweep)

    geometry = scenario["geometry"]
    sensor_range = float(geometry["sensor_range"])
    ray_count = int(geometry["ray_count"])
    sample_resolution = float(sweep["area_sample_resolution"])
    narrow_clearance = float(geometry["narrow_clearance"])
    visibility_weights = {
        str(key): float(value)
        for key, value in geometry["visibility_weights"].items()
    }
    forward_window = geometry["forward_window"]
    obstacle_inflation = float(geometry["obstacle_inflation"])
    offsets = formation["offsets"]
    edges = formation["candidate_edges"]
    centre_config = sweep["formation_center"]
    centre_x_values = list(inclusive_values(
        float(centre_config["x_start"]),
        float(centre_config["x_end"]),
        float(centre_config["x_step"]),
    ))
    centre_y = float(centre_config["y"])
    centre_z = float(centre_config["z"])
    radii = [float(value) for value in sweep["pillar_radii"]]
    pillar_y_offsets = [float(value) for value in sweep["pillar_y_offsets"]]

    fieldnames = [
        "snapshot_id", "case_id", "pillar_radius", "inflated_pillar_radius",
        "pillar_y", "center_x", "center_y", "center_z",
        "uav_i", "uav_j", "uav_i_x", "uav_i_y", "uav_j_x", "uav_j_y",
        "uav_i_in_obstacle", "uav_j_in_obstacle",
        "area_i", "area_j", "intersection_area", "union_area",
        "overlap_min", "overlap_iou", "clearance", "visibility",
        "visibility_weight", "composite_min", "composite_iou",
        "forward_area_i", "forward_area_j", "forward_intersection_area",
        "forward_union_area", "forward_overlap_min", "forward_overlap_iou",
        "forward_composite_min", "forward_composite_iou",
    ]
    args.csv_out.parent.mkdir(parents=True, exist_ok=True)
    case_count = len(radii) * len(pillar_y_offsets)
    snapshot_count = 0
    row_count = 0

    with args.csv_out.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        case_index = 0
        for radius in radii:
            for pillar_y in pillar_y_offsets:
                case_id = f"r{radius:.2f}_y{pillar_y:+.2f}"
                pillar = Pillar(Vec2(0.0, pillar_y), radius + obstacle_inflation)
                for centre_x in centre_x_values:
                    snapshot_id = snapshot_count
                    snapshot_count += 1
                    positions = [
                        Vec2(centre_x + float(offset[0]), centre_y + float(offset[1]))
                        for offset in offsets
                    ]
                    spaces = [
                        build_star_space(position, [pillar], sensor_range, ray_count)
                        for position in positions
                    ]
                    points, memberships, cell_area = sample_grid_memberships(
                        spaces, sample_resolution
                    )
                    for first_raw, second_raw in edges:
                        first, second = int(first_raw), int(second_raw)
                        score = pair_score(
                            spaces[first], spaces[second],
                            memberships[first], memberships[second], cell_area,
                            [pillar], narrow_clearance, visibility_weights,
                        )
                        midpoint = Vec2(
                            0.5 * (positions[first].x + positions[second].x),
                            0.5 * (positions[first].y + positions[second].y),
                        )
                        mask = forward_window_mask(
                            points,
                            midpoint,
                            float(forward_window["heading_deg"]),
                            float(forward_window["longitudinal_start"]),
                            float(forward_window["length"]),
                            float(forward_window["half_width"]),
                        )
                        forward_overlap = sampled_overlap(
                            memberships[first], memberships[second], cell_area, mask
                        )
                        writer.writerow({
                            "snapshot_id": snapshot_id,
                            "case_id": case_id,
                            "pillar_radius": radius,
                            "inflated_pillar_radius": pillar.radius,
                            "pillar_y": pillar_y,
                            "center_x": centre_x,
                            "center_y": centre_y,
                            "center_z": centre_z,
                            "uav_i": first,
                            "uav_j": second,
                            "uav_i_x": positions[first].x,
                            "uav_i_y": positions[first].y,
                            "uav_j_x": positions[second].x,
                            "uav_j_y": positions[second].y,
                            "uav_i_in_obstacle": int(spaces[first].center_in_obstacle),
                            "uav_j_in_obstacle": int(spaces[second].center_in_obstacle),
                            "area_i": score.area_a,
                            "area_j": score.area_b,
                            "intersection_area": score.intersection_area,
                            "union_area": score.union_area,
                            "overlap_min": score.overlap_min,
                            "overlap_iou": score.overlap_iou,
                            "clearance": score.clearance,
                            "visibility": score.visibility,
                            "visibility_weight": score.visibility_weight,
                            "composite_min": score.composite_min,
                            "composite_iou": score.composite_iou,
                            "forward_area_i": forward_overlap.area_a,
                            "forward_area_j": forward_overlap.area_b,
                            "forward_intersection_area": forward_overlap.intersection_area,
                            "forward_union_area": forward_overlap.union_area,
                            "forward_overlap_min": forward_overlap.overlap_min,
                            "forward_overlap_iou": forward_overlap.overlap_iou,
                            "forward_composite_min": (
                                forward_overlap.overlap_min * score.visibility_weight
                            ),
                            "forward_composite_iou": (
                                forward_overlap.overlap_iou * score.visibility_weight
                            ),
                        })
                        row_count += 1
                case_index += 1
                print(
                    f"case {case_index}/{case_count} complete: {case_id}",
                    flush=True,
                )

    summary = {
        "name": sweep["name"],
        "preview_horizon": 0.0,
        "area_sample_resolution": sample_resolution,
        "pillar_radii": radii,
        "pillar_y_offsets": pillar_y_offsets,
        "x_positions_per_case": len(centre_x_values),
        "cases": case_count,
        "snapshots": snapshot_count,
        "candidate_edges": len(edges),
        "rows": row_count,
        "csv": str(args.csv_out.resolve()),
    }
    if args.summary_out:
        args.summary_out.parent.mkdir(parents=True, exist_ok=True)
        with args.summary_out.open("w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)
            stream.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
