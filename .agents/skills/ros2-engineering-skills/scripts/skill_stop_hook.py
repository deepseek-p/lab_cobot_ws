#!/usr/bin/env python3
"""Skills 2.0 Stop Hook — Post-execution validation for ros2-engineering-skills.

This hook runs when the skill execution stops. It validates that any generated
ROS 2 artifacts (packages, launch files, QoS configurations) conform to the
skill's engineering principles.

When the workspace is a git repository, validation is scoped to files that
are modified or untracked according to git — i.e. plausibly touched by this
session. A pre-existing broken launch file that is committed and untouched
must not make every Stop fail forever in a repo the skill never modified.
Outside a git repository (or if git is unavailable) all discovered files
are validated, as before.

Exit codes:
    0 — All checks passed
    1 — Validation issues found (reported to stdout as JSON)
"""

import json
import os
import subprocess
import sys
import ast


# Maximum directory depth to walk (relative to workspace root).
# Keeps scan cost bounded for large workspaces with deeply nested vendor trees.
_MAX_SCAN_DEPTH = 6

# Directory names to always skip (in addition to hidden dirs).
_SKIP_DIRS = frozenset((
    'build', 'install', 'log', 'node_modules', '__pycache__',
    '.git', '.svn', 'venv', '.venv', 'third_party', 'vendor',
))


def _should_skip(dirpath, workspace):
    """Return True if *dirpath* should be pruned from the walk."""
    rel = os.path.relpath(dirpath, workspace)
    if rel == '.':
        return False  # never skip the workspace root itself
    parts = rel.split(os.sep)
    if len(parts) > _MAX_SCAN_DEPTH:
        return True
    return any(p.startswith('.') or p in _SKIP_DIRS for p in parts)


def find_generated_launch_files(workspace):
    """Find all launch files in the workspace (depth-limited).

    Matches both official Python launch naming conventions
    (*.launch.py and *_launch.py).
    """
    launch_files = []
    for root, dirs, files in os.walk(workspace):
        if _should_skip(root, workspace):
            dirs.clear()  # prune subtree
            continue
        # In-place prune to avoid descending into skippable children
        dirs[:] = [d for d in dirs
                   if not d.startswith('.') and d not in _SKIP_DIRS]
        for f in files:
            if f.endswith('.launch.py') or f.endswith('_launch.py'):
                launch_files.append(os.path.join(root, f))
    return launch_files


def validate_launch_file_syntax(filepath):
    """Check that a launch file is valid Python and has generate_launch_description."""
    issues = []
    try:
        with open(filepath, 'r', encoding='utf-8') as fh:
            source = fh.read()
        tree = ast.parse(source, filename=filepath)
        func_names = [
            node.name for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef)
        ]
        if 'generate_launch_description' not in func_names:
            issues.append({
                'file': filepath,
                'severity': 'error',
                'message': 'Missing generate_launch_description function',
            })
    except SyntaxError as e:
        issues.append({
            'file': filepath,
            'severity': 'error',
            'message': f'Syntax error: {e}',
        })
    except OSError:
        pass  # File may have been removed during session
    return issues


def validate_package_xml(filepath):
    """Check that a package.xml uses format 3 and has required elements."""
    issues = []
    try:
        import xml.etree.ElementTree as ET
        tree = ET.parse(filepath)
        root = tree.getroot()
        fmt = root.attrib.get('format', '')
        if fmt != '3':
            issues.append({
                'file': filepath,
                'severity': 'warning',
                'message': f'package.xml uses format {fmt}, recommend format 3',
            })
        if root.find('name') is None:
            issues.append({
                'file': filepath,
                'severity': 'error',
                'message': 'package.xml missing <name> element',
            })
        if root.find('license') is None:
            issues.append({
                'file': filepath,
                'severity': 'warning',
                'message': 'package.xml missing <license> element',
            })
    except Exception as e:
        issues.append({
            'file': filepath,
            'severity': 'error',
            'message': f'Failed to parse package.xml: {e}',
        })
    return issues


def find_package_xmls(workspace):
    """Find all package.xml files in the workspace (depth-limited)."""
    results = []
    for root, dirs, files in os.walk(workspace):
        if _should_skip(root, workspace):
            dirs.clear()
            continue
        dirs[:] = [d for d in dirs
                   if not d.startswith('.') and d not in _SKIP_DIRS]
        for f in files:
            if f == 'package.xml':
                results.append(os.path.join(root, f))
    return results


def _resolve_workspace():
    """Pick the workspace path to scan, preferring explicit signals.

    Real Claude Code (verified 2026-05-21) sends Stop-event payloads via
    stdin including a `cwd` field naming the workspace root. We prefer that
    over `os.getcwd()` because the hook process may be invoked from a
    different working directory than the user's actual project root.

    Resolution order:
      1. SKILL_WORKSPACE env var (explicit override, used by pytest)
      2. stdin JSON payload `cwd` (real Claude Code)
      3. CLAUDE_PROJECT_DIR env var (Claude Code sets this for hooks)
      4. os.getcwd() fallback
    """
    explicit = os.environ.get('SKILL_WORKSPACE')
    if explicit:
        return explicit

    if not sys.stdin.isatty():
        try:
            raw = sys.stdin.read()
            if raw.strip():
                payload = json.loads(raw)
                if isinstance(payload, dict):
                    cwd = payload.get('cwd')
                    if cwd and os.path.isdir(cwd):
                        return cwd
        except (json.JSONDecodeError, OSError, Exception):  # noqa: BLE001
            pass

    project_dir = os.environ.get('CLAUDE_PROJECT_DIR')
    if project_dir and os.path.isdir(project_dir):
        return project_dir

    return os.getcwd()


def _git_touched_paths(workspace):
    """Return real paths of git-modified/untracked files, or None.

    None means the modification set is unknown (not a git repository, git
    missing, or git failed) — the caller then validates everything found,
    preserving the pre-git behaviour.
    """
    try:
        proc = subprocess.run(
            ['git', '-C', workspace, 'status', '--porcelain',
             '--untracked-files=all', '--no-renames'],
            capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return None
    if proc.returncode != 0:
        return None
    paths = set()
    for line in proc.stdout.splitlines():
        # Porcelain v1: two status chars, a space, then the path
        # (quoted if it contains special characters).
        p = line[3:].strip()
        if p.startswith('"') and p.endswith('"'):
            p = p[1:-1]
        if p:
            paths.add(os.path.realpath(os.path.join(workspace, p)))
    return paths


def main():
    workspace = _resolve_workspace()
    all_issues = []

    launch_files = find_generated_launch_files(workspace)
    package_xmls = find_package_xmls(workspace)

    # Scope validation to files this session plausibly touched (see module
    # docstring). Unknown modification set -> validate everything.
    touched = _git_touched_paths(workspace)
    if touched is not None:
        launch_files = [f for f in launch_files
                        if os.path.realpath(f) in touched]
        package_xmls = [f for f in package_xmls
                        if os.path.realpath(f) in touched]

    for lf in launch_files:
        all_issues.extend(validate_launch_file_syntax(lf))

    for px in package_xmls:
        all_issues.extend(validate_package_xml(px))

    result = {
        'hook': 'ros2-engineering-skills:stop',
        'version': '1.1.0',
        'issues_count': len(all_issues),
        'issues': all_issues,
        'status': 'fail' if any(
            i['severity'] == 'error' for i in all_issues
        ) else 'pass',
    }

    # --- Execution log pattern ---
    # Append a summary to .skill-runs.log so the next session can see
    # what was validated and what issues were found last time.
    log_path = os.path.join(workspace, '.skill-runs.log')
    try:
        from datetime import datetime, timezone
        log_entry = {
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'status': result['status'],
            'issues_count': result['issues_count'],
            'launch_files_checked': len(launch_files),
            'package_xmls_checked': len(package_xmls),
            'error_summaries': [
                i['message'] for i in all_issues if i['severity'] == 'error'
            ][:5],  # keep log concise
        }
        with open(log_path, 'a', encoding='utf-8') as lf:
            lf.write(json.dumps(log_entry) + '\n')
    except OSError:
        pass  # logging is best-effort

    print(json.dumps(result, indent=2))
    sys.exit(1 if result['status'] == 'fail' else 0)


if __name__ == '__main__':
    main()
