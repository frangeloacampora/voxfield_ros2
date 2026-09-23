#!/usr/bin/env python3
"""Convert ROS 1 rosparam YAML files to ROS 2 parameter-file format, in place.

For each input file this:
  1. Loads it with a full YAML loader, so `&anchor` / `*alias` references
     resolve to their concrete values.
  2. Flattens every 4x4 (or any NxM) list-of-lists of numbers -- the
     transform matrices such as `T_B_C`, `T_B_D`, `T_C_CH` -- into a single
     row-major list of floats, since nested lists aren't a valid ROS 2
     parameter type. Every element is forced to a Python float (`1` ->
     `1.0`) so the ROS 2 parameter parser infers `double_array`, not
     `integer_array`.
  3. Wraps the result as `/**:\n  ros__parameters:\n    ...`, so the file
     applies regardless of the node's name or namespace (matching the old
     files, which had no node-name qualifier either).
  4. Re-verifies its own output: every scalar must round-trip unchanged and
     every matrix must flatten to the same numbers in the same order as the
     original, before anything is written to disk.

Anchors/aliases collapse into plain repeated values (ROS 2's YAML parser,
rcl_yaml_param_parser, does not support them), and scalar types are left
as PyYAML infers them (ints stay ints, floats stay floats) -- the
project's tolerant param-loading helper (voxfield::param, see D6 in
ROS2_PORT_PLAN.md) coerces int<->double at load time, so this script does
not need to force every scalar to a particular type.

Comment preservation is out of scope (the plan calls it a nice-to-have);
this uses plain PyYAML, so comments are dropped on conversion.

Usage:
    convert_ros1_params.py <file1.yaml> [<file2.yaml> ...]
"""
import argparse
import sys

import yaml


def is_matrix(value):
    """True if `value` is a non-empty list of lists of numbers."""
    return (
        isinstance(value, list)
        and len(value) > 0
        and all(isinstance(row, list) and len(row) > 0 for row in value)
        and all(isinstance(x, (int, float)) for row in value for x in row)
    )


def flatten_matrix(value):
    return [float(x) for row in value for x in row]


def convert(node):
    """Recursively flatten matrices found anywhere in a parameter tree."""
    if is_matrix(node):
        return flatten_matrix(node)
    if isinstance(node, dict):
        return {key: convert(value) for key, value in node.items()}
    return node


def verify(original, converted):
    """Raise if `converted` isn't a faithful transform of `original`."""
    if is_matrix(original):
        expected = flatten_matrix(original)
        if converted != expected:
            raise AssertionError(
                f"matrix flattened incorrectly: {original} -> {converted}, "
                f"expected {expected}")
        return
    if isinstance(original, dict):
        if not isinstance(converted, dict):
            raise AssertionError(f"expected a mapping, got {converted!r}")
        if original.keys() != converted.keys():
            raise AssertionError(
                f"key set changed: {sorted(original.keys())} -> "
                f"{sorted(converted.keys())}")
        for key, value in original.items():
            verify(value, converted[key])
        return
    if original != converted:
        raise AssertionError(f"value changed: {original!r} -> {converted!r}")


def convert_file(path):
    with open(path) as f:
        original = yaml.safe_load(f)
    if original is None:
        original = {}
    if not isinstance(original, dict):
        raise ValueError(f"{path}: top level is not a mapping")

    if list(original.keys()) == ["/**"]:
        print(f"  already in ROS 2 format, skipping: {path}")
        return

    converted_params = convert(original)
    verify(original, converted_params)

    wrapped = {"/**": {"ros__parameters": converted_params}}
    out = yaml.dump(wrapped, sort_keys=False, default_flow_style=False)

    with open(path, "w") as f:
        f.write(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="+", help="YAML files to convert in place")
    args = parser.parse_args()

    for path in args.files:
        print(f"Converting {path}")
        convert_file(path)


if __name__ == "__main__":
    sys.exit(main())
