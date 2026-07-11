"""Refined position selection tests."""
import ast
import importlib
from pathlib import Path


def _module():
    return importlib.import_module("lab_cobot_manipulation.refine_select")


def test_none_refinement_keeps_coarse_position():
    selected, used_refine, reason = _module().select_refined_position(
        [0.1, 0.2, 0.3],
        None,
    )

    assert selected == [0.1, 0.2, 0.3]
    assert used_refine is False
    assert reason == "no_fresh_sample"


def test_small_correction_uses_refined_position():
    selected, used_refine, reason = _module().select_refined_position(
        [0.0, 0.0, 0.0],
        [0.03, 0.0, 0.0],
    )

    assert selected == [0.03, 0.0, 0.0]
    assert used_refine is True
    assert reason == "ok"


def test_large_correction_keeps_coarse_position():
    selected, used_refine, reason = _module().select_refined_position(
        [0.0, 0.0, 0.0],
        [0.08, 0.0, 0.0],
    )

    assert selected == [0.0, 0.0, 0.0]
    assert used_refine is False
    assert reason == "correction_exceeds_gate"


def test_correction_equal_to_gate_is_accepted():
    module = _module()

    selected, used_refine, reason = module.select_refined_position(
        [0.0, 0.0, 0.0],
        [module.REFINE_MAX_CORRECTION_M, 0.0, 0.0],
    )

    assert selected == [0.05, 0.0, 0.0]
    assert used_refine is True
    assert reason == "ok"


def test_selection_does_not_mutate_inputs():
    coarse = [0.1, 0.2, 0.3]
    refined = [0.12, 0.2, 0.3]
    coarse_before = list(coarse)
    refined_before = list(refined)

    _module().select_refined_position(coarse, refined)

    assert coarse == coarse_before
    assert refined == refined_before


def test_module_top_level_imports_only_standard_library():
    module_path = (
        Path(__file__).resolve().parents[1]
        / "lab_cobot_manipulation"
        / "refine_select.py"
    )
    assert module_path.exists()
    tree = ast.parse(module_path.read_text(encoding="utf-8"))
    imports = {
        alias.name.split(".")[0]
        for node in tree.body
        if isinstance(node, ast.Import)
        for alias in node.names
    }
    imports.update(
        node.module.split(".")[0]
        for node in tree.body
        if isinstance(node, ast.ImportFrom) and node.module is not None
    )

    assert imports <= {"math"}
