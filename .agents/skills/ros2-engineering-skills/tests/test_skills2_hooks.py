"""Tests for Skills 2.0 hook scripts — validates hook execution and output.

These tests ensure:
1. Hook scripts are executable and produce valid JSON output
2. Stop hook correctly validates ROS 2 artifacts
3. PreToolUse hook detects anti-patterns
4. Hooks return correct exit codes
"""

import json
import os
import subprocess
import sys

SCRIPTS_DIR = os.path.join(os.path.dirname(__file__), '..', 'scripts')

sys.path.insert(0, SCRIPTS_DIR)
from skill_stop_hook import (
    find_generated_launch_files,
    validate_launch_file_syntax,
    validate_package_xml,
    find_package_xmls,
    _git_touched_paths,
)
from skill_validate_hook import (
    check_content,
    check_file,
    _check_dangerous_commands,
    ANTIPATTERN_CHECKS,
    CHECKABLE_EXTENSIONS,
    DANGEROUS_COMMAND_PATTERNS,
)


class TestStopHookLaunchValidation:
    """Test the stop hook's launch file validation."""

    def test_valid_launch_file(self, tmp_path):
        launch = tmp_path / 'test.launch.py'
        launch.write_text(
            'from launch import LaunchDescription\n'
            'def generate_launch_description():\n'
            '    return LaunchDescription([])\n'
        )
        issues = validate_launch_file_syntax(str(launch))
        assert len(issues) == 0

    def test_missing_generate_function(self, tmp_path):
        launch = tmp_path / 'bad.launch.py'
        launch.write_text(
            'from launch import LaunchDescription\n'
            'def create_nodes():\n'
            '    return LaunchDescription([])\n'
        )
        issues = validate_launch_file_syntax(str(launch))
        assert len(issues) == 1
        assert issues[0]['severity'] == 'error'
        assert 'generate_launch_description' in issues[0]['message']

    def test_syntax_error(self, tmp_path):
        launch = tmp_path / 'syntax.launch.py'
        launch.write_text('def broken(\n')
        issues = validate_launch_file_syntax(str(launch))
        assert len(issues) == 1
        assert issues[0]['severity'] == 'error'
        assert 'yntax' in issues[0]['message']

    def test_nonexistent_file(self):
        issues = validate_launch_file_syntax('/nonexistent/file.launch.py')
        assert len(issues) == 0  # File read errors are silently skipped

    def test_find_launch_files(self, tmp_path):
        launch_dir = tmp_path / 'pkg' / 'launch'
        launch_dir.mkdir(parents=True)
        (launch_dir / 'a.launch.py').write_text('# launch')
        (launch_dir / 'b.launch.py').write_text('# launch')
        # *_launch.py is the other official naming convention
        (launch_dir / 'c_launch.py').write_text('# launch')
        (tmp_path / 'helpers.py').write_text('# not a launch')
        files = find_generated_launch_files(str(tmp_path))
        assert len(files) == 3

    def test_find_launch_files_skips_hidden(self, tmp_path):
        hidden = tmp_path / '.hidden' / 'launch'
        hidden.mkdir(parents=True)
        (hidden / 'skip.launch.py').write_text('# skip')
        files = find_generated_launch_files(str(tmp_path))
        assert len(files) == 0

    def test_find_launch_files_skips_build(self, tmp_path):
        build = tmp_path / 'build' / 'pkg' / 'launch'
        build.mkdir(parents=True)
        (build / 'skip.launch.py').write_text('# skip')
        files = find_generated_launch_files(str(tmp_path))
        assert len(files) == 0


class TestStopHookPackageXmlValidation:
    """Test the stop hook's package.xml validation."""

    def test_valid_package_xml(self, tmp_path):
        pkg_xml = tmp_path / 'package.xml'
        pkg_xml.write_text(
            '<?xml version="1.0"?>\n'
            '<package format="3">\n'
            '  <name>test_pkg</name>\n'
            '  <version>0.1.0</version>\n'
            '  <description>Test</description>\n'
            '  <maintainer email="a@b.c">Test</maintainer>\n'
            '  <license>Apache-2.0</license>\n'
            '</package>\n'
        )
        issues = validate_package_xml(str(pkg_xml))
        assert len(issues) == 0

    def test_old_format_warns(self, tmp_path):
        pkg_xml = tmp_path / 'package.xml'
        pkg_xml.write_text(
            '<?xml version="1.0"?>\n'
            '<package format="2">\n'
            '  <name>test_pkg</name>\n'
            '  <license>Apache-2.0</license>\n'
            '</package>\n'
        )
        issues = validate_package_xml(str(pkg_xml))
        warnings = [i for i in issues if i['severity'] == 'warning']
        assert any('format' in i['message'] for i in warnings)

    def test_missing_name_errors(self, tmp_path):
        pkg_xml = tmp_path / 'package.xml'
        pkg_xml.write_text(
            '<?xml version="1.0"?>\n'
            '<package format="3">\n'
            '  <license>Apache-2.0</license>\n'
            '</package>\n'
        )
        issues = validate_package_xml(str(pkg_xml))
        errors = [i for i in issues if i['severity'] == 'error']
        assert any('name' in i['message'] for i in errors)

    def test_missing_license_warns(self, tmp_path):
        pkg_xml = tmp_path / 'package.xml'
        pkg_xml.write_text(
            '<?xml version="1.0"?>\n'
            '<package format="3">\n'
            '  <name>test_pkg</name>\n'
            '</package>\n'
        )
        issues = validate_package_xml(str(pkg_xml))
        warnings = [i for i in issues if i['severity'] == 'warning']
        assert any('license' in i['message'] for i in warnings)

    def test_invalid_xml_errors(self, tmp_path):
        pkg_xml = tmp_path / 'package.xml'
        pkg_xml.write_text('not xml at all')
        issues = validate_package_xml(str(pkg_xml))
        assert any(i['severity'] == 'error' for i in issues)

    def test_find_package_xmls(self, tmp_path):
        (tmp_path / 'pkg_a').mkdir()
        (tmp_path / 'pkg_a' / 'package.xml').write_text('<package/>')
        (tmp_path / 'pkg_b').mkdir()
        (tmp_path / 'pkg_b' / 'package.xml').write_text('<package/>')
        files = find_package_xmls(str(tmp_path))
        assert len(files) == 2

    def test_find_package_xmls_skips_build(self, tmp_path):
        build = tmp_path / 'build' / 'pkg'
        build.mkdir(parents=True)
        (build / 'package.xml').write_text('<package/>')
        files = find_package_xmls(str(tmp_path))
        assert len(files) == 0


class TestStopHookCLI:
    """Test the stop hook as a CLI command."""

    def test_clean_workspace_passes(self, tmp_path):
        result = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_stop_hook.py')],
            capture_output=True, text=True,
            env={**os.environ, 'SKILL_WORKSPACE': str(tmp_path)},
        )
        assert result.returncode == 0
        data = json.loads(result.stdout)
        assert data['status'] == 'pass'
        assert data['issues_count'] == 0

    def test_workspace_with_valid_artifacts(self, tmp_path):
        (tmp_path / 'launch').mkdir()
        (tmp_path / 'launch' / 'test.launch.py').write_text(
            'from launch import LaunchDescription\n'
            'def generate_launch_description():\n'
            '    return LaunchDescription([])\n'
        )
        (tmp_path / 'package.xml').write_text(
            '<?xml version="1.0"?>\n'
            '<package format="3">\n'
            '  <name>test</name>\n'
            '  <license>Apache-2.0</license>\n'
            '</package>\n'
        )
        result = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_stop_hook.py')],
            capture_output=True, text=True,
            env={**os.environ, 'SKILL_WORKSPACE': str(tmp_path)},
        )
        assert result.returncode == 0
        data = json.loads(result.stdout)
        assert data['status'] == 'pass'

    def test_workspace_with_errors_fails(self, tmp_path):
        (tmp_path / 'launch').mkdir()
        (tmp_path / 'launch' / 'bad.launch.py').write_text(
            'from launch import LaunchDescription\n'
            'def wrong_name():\n'
            '    return LaunchDescription([])\n'
        )
        (tmp_path / 'package.xml').write_text(
            '<?xml version="1.0"?>\n'
            '<package format="3">\n'
            '  <license>Apache-2.0</license>\n'
            '</package>\n'
        )
        result = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_stop_hook.py')],
            capture_output=True, text=True,
            env={**os.environ, 'SKILL_WORKSPACE': str(tmp_path)},
        )
        assert result.returncode == 1
        data = json.loads(result.stdout)
        assert data['status'] == 'fail'
        assert data['issues_count'] >= 1

    def _git(self, cwd, *args):
        return subprocess.run(
            ['git', '-C', str(cwd), *args],
            capture_output=True, text=True, check=True,
            env={**os.environ,
                 'GIT_AUTHOR_NAME': 't', 'GIT_AUTHOR_EMAIL': 't@t',
                 'GIT_COMMITTER_NAME': 't', 'GIT_COMMITTER_EMAIL': 't@t'},
        )

    def test_committed_broken_file_does_not_block_stop(self, tmp_path):
        # A pre-existing broken launch file, committed and untouched by the
        # session, must not fail the Stop hook forever.
        (tmp_path / 'launch').mkdir()
        (tmp_path / 'launch' / 'bad.launch.py').write_text(
            'def wrong_name():\n    pass\n')
        self._git(tmp_path, 'init', '-q')
        self._git(tmp_path, 'add', '-A')
        self._git(tmp_path, 'commit', '-qm', 'x')
        result = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_stop_hook.py')],
            capture_output=True, text=True,
            env={**os.environ, 'SKILL_WORKSPACE': str(tmp_path)},
        )
        assert result.returncode == 0
        data = json.loads(result.stdout)
        assert data['status'] == 'pass'

    def test_git_touched_paths_non_repo_returns_none(self, tmp_path):
        assert _git_touched_paths(str(tmp_path)) is None

    def test_git_touched_paths_lists_modified_and_untracked(self, tmp_path):
        (tmp_path / 'tracked.txt').write_text('v1')
        self._git(tmp_path, 'init', '-q')
        self._git(tmp_path, 'add', '-A')
        self._git(tmp_path, 'commit', '-qm', 'x')
        (tmp_path / 'tracked.txt').write_text('v2')
        (tmp_path / 'new file.txt').write_text('n')  # quoted in porcelain
        touched = _git_touched_paths(str(tmp_path))
        assert touched is not None
        assert os.path.realpath(str(tmp_path / 'tracked.txt')) in touched
        assert os.path.realpath(str(tmp_path / 'new file.txt')) in touched

    def test_untracked_broken_file_still_fails(self, tmp_path):
        # In a git workspace, a broken file the session just created
        # (untracked) is still validated and fails the hook.
        self._git(tmp_path, 'init', '-q')
        (tmp_path / 'launch').mkdir()
        (tmp_path / 'launch' / 'new.launch.py').write_text(
            'def wrong_name():\n    pass\n')
        result = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_stop_hook.py')],
            capture_output=True, text=True,
            env={**os.environ, 'SKILL_WORKSPACE': str(tmp_path)},
        )
        assert result.returncode == 1
        data = json.loads(result.stdout)
        assert data['status'] == 'fail'

    def test_output_is_valid_json(self, tmp_path):
        result = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_stop_hook.py')],
            capture_output=True, text=True,
            env={**os.environ, 'SKILL_WORKSPACE': str(tmp_path)},
        )
        data = json.loads(result.stdout)
        assert 'hook' in data
        assert 'version' in data
        assert 'issues_count' in data
        assert 'issues' in data
        assert 'status' in data


class TestValidateHookAntiPatterns:
    """Test the PreToolUse hook's anti-pattern detection."""

    def test_detects_time_sleep(self):
        issues = check_content('time.sleep(5)', 'test.py')
        assert len(issues) >= 1
        assert any('sleep' in i['message'] for i in issues)

    def test_detects_spin_until_future_complete(self):
        issues = check_content(
            'rclpy.spin_until_future_complete(node, future)', 'test.py')
        assert len(issues) >= 1
        assert any('spin_until_future_complete' in i['message'] for i in issues)

    def test_detects_global_variables(self):
        issues = check_content('global node_state', 'test.py')
        assert len(issues) >= 1
        assert any('Global' in i['message'] for i in issues)

    def test_detects_ros_localhost_only(self):
        issues = check_content(
            'os.environ["ROS_LOCALHOST_ONLY"] = "1"', 'test.py')
        assert len(issues) >= 1
        assert any('ROS_LOCALHOST_ONLY' in i['message'] for i in issues)

    def test_detects_deprecated_node_executable(self):
        # Deprecated launch_ros kwarg - check only fires on .launch.py files
        # (false-positive guard added 2026-05; see ANTIPATTERN_CHECKS).
        issues = check_content(
            'Node(node_executable="my_node")', 'bringup.launch.py')
        assert len(issues) >= 1
        assert any('deprecated' in i['message'] for i in issues)

    def test_detects_deprecated_node_name(self):
        issues = check_content(
            'Node(node_name="my_node")', 'bringup.launch.py')
        assert len(issues) >= 1
        assert any('deprecated' in i['message'] for i in issues)

    def test_detects_deprecated_node_namespace(self):
        issues = check_content(
            'Node(node_namespace="/ns")', 'bringup.launch.py')
        assert len(issues) >= 1
        assert any('deprecated' in i['message'] for i in issues)

    def test_deprecated_kwargs_not_flagged_in_non_launch_file(self):
        """Regression: anti-pattern checks for launch-only kwargs must NOT
        fire on regular Python files. A dataclass or test fixture happening
        to use a `node_name=` parameter is not a deprecated launch_ros API."""
        for kwarg in ('node_name', 'node_executable', 'node_namespace'):
            src = (f'class Robot:\n'
                   f'    {kwarg} = "default_robot_name"\n')
            issues = check_content(src, 'robot.py')
            assert all('deprecated' not in i['message'] for i in issues), (
                f'{kwarg} should not be flagged in non-.launch.py files; '
                f'got: {issues}'
            )

    def test_global_false_positive_avoided_in_string_literal(self):
        """Regression: 'global' inside a string literal must NOT be flagged
        as a global statement. Only the `global X` Python statement counts."""
        src = 'config = {"global": True, "scope": "process"}\n'
        issues = check_content(src, 'cfg.py')
        assert all('Global variables' not in i['message'] for i in issues), (
            f'string-literal global must not be flagged; got: {issues}'
        )

    def test_global_statement_still_detected_at_line_start(self):
        """The legitimate Python `global` statement should still be caught."""
        src = ('def f():\n'
               '    global state\n'
               '    state = 1\n')
        issues = check_content(src, 'cfg.py')
        assert any('Global variables' in i['message'] for i in issues), (
            f'real global statement must still be flagged; got: {issues}'
        )

    def test_global_identifier_not_at_line_start_not_flagged(self):
        """An identifier containing 'global' (e.g. `global_var`) is not a
        `global` statement and must not trigger the warning."""
        src = ('global_var = 1\n'
               'my_global = "hi"\n'
               'x = global_settings.get("x")\n')
        issues = check_content(src, 'mod.py')
        assert all('Global variables' not in i['message'] for i in issues), (
            f'identifier with "global" prefix must not be flagged; '
            f'got: {issues}'
        )

    def test_clean_code_no_issues(self):
        clean_code = (
            'import rclpy\n'
            'class MyNode(Node):\n'
            '    def __init__(self):\n'
            '        super().__init__("my_node")\n'
        )
        issues = check_content(clean_code, 'test.py')
        assert len(issues) == 0

    def test_docstring_with_antipattern_is_flagged(self):
        # Documented limitation: the comment-skipping heuristic only handles
        # `#` and `//` single-line comments, not Python triple-quoted strings.
        # A docstring that mentions `time.sleep()` is expected to trigger a
        # warning. This test pins that behavior so the docstring in
        # skill_validate_hook.py stays accurate.
        code = (
            'def f():\n'
            '    """Avoid time.sleep(5) in ROS 2 nodes."""\n'
            '    return 1\n'
        )
        issues = check_content(code, 'test.py')
        assert any('time.sleep' in i['message'] for i in issues)

    def test_floor_division_not_treated_as_comment_in_python(self):
        # `//` is floor division in Python, not a comment; a match after it
        # on the same line must still be reported.
        code = 'n = total // 2; time.sleep(1)\n'
        issues = check_content(code, 'test.py')
        assert any('time.sleep' in i['message'] for i in issues)

    def test_double_slash_comment_skipped_in_cpp(self):
        code = 'int x = 1; // calls time.sleep(1) in the python port\n'
        issues = check_content(code, 'test.cpp')
        assert len(issues) == 0

    def test_hash_comment_skipped_in_python(self):
        code = 'x = 1  # time.sleep(1) would be wrong here\n'
        issues = check_content(code, 'test.py')
        assert len(issues) == 0

    def test_deprecated_kwargs_flagged_in_underscore_launch_file(self):
        code = "node_name='talker'\n"
        issues = check_content(code, 'robot_launch.py')
        assert any('node_name' in i['message'] for i in issues)

    def test_check_file_returns_empty_for_non_checkable(self, tmp_path):
        f = tmp_path / 'test.yaml'
        f.write_text('key: value')
        issues = check_file(str(f))
        assert len(issues) == 0

    def test_check_file_checks_python(self, tmp_path):
        f = tmp_path / 'test.py'
        f.write_text('time.sleep(1)')
        issues = check_file(str(f))
        assert len(issues) >= 1

    def test_check_file_checks_cpp(self, tmp_path):
        f = tmp_path / 'test.cpp'
        f.write_text('// clean C++ code\n')
        issues = check_file(str(f))
        assert len(issues) == 0

    def test_check_file_nonexistent(self):
        issues = check_file('/nonexistent/file.py')
        assert len(issues) == 0

    def test_antipattern_checks_non_empty(self):
        assert len(ANTIPATTERN_CHECKS) >= 5

    def test_checkable_extensions(self):
        assert '.py' in CHECKABLE_EXTENSIONS
        assert '.cpp' in CHECKABLE_EXTENSIONS
        assert '.hpp' in CHECKABLE_EXTENSIONS


class TestDangerousCommandDetection:
    """Test dangerous command detection in the PreToolUse hook."""

    def test_rm_rf_root(self):
        issues = _check_dangerous_commands('rm -rf /')
        assert len(issues) >= 1
        assert any('root' in i['message'].lower() for i in issues)

    def test_rm_rf_root_star(self):
        issues = _check_dangerous_commands('rm -rf /*')
        assert len(issues) >= 1

    def test_rm_rf_opt_ros(self):
        issues = _check_dangerous_commands('rm -rf /opt/ros')
        assert len(issues) >= 1
        assert any('ROS' in i['message'] for i in issues)

    def test_rm_rf_system_dirs(self):
        for d in ['/usr', '/bin', '/etc', '/var', '/boot', '/lib']:
            issues = _check_dangerous_commands(f'rm -rf {d}')
            assert len(issues) >= 1, f"Should detect rm -rf {d}"

    def test_rm_rf_home(self):
        issues = _check_dangerous_commands('rm -rf ~')
        assert len(issues) >= 1

    def test_mkfs_detected(self):
        issues = _check_dangerous_commands('mkfs.ext4 /dev/sda1')
        assert len(issues) >= 1
        assert any('mkfs' in i['message'] for i in issues)

    def test_dd_to_disk(self):
        issues = _check_dangerous_commands('dd if=/dev/zero of=/dev/sda')
        assert len(issues) >= 1
        assert any('dd' in i['message'].lower() for i in issues)

    def test_chmod_777_root(self):
        issues = _check_dangerous_commands('chmod -R 777 /')
        assert len(issues) >= 1
        assert any('chmod' in i['message'] for i in issues)

    def test_rm_rf_system_dir_with_trailing_slash_or_star(self):
        for cmd in ['rm -rf /usr/', 'rm -rf /var/*']:
            issues = _check_dangerous_commands(cmd)
            assert len(issues) >= 1, f"Should detect {cmd}"

    def test_rm_rf_in_compound_command(self):
        for cmd in ['rm -rf /; echo done', 'rm -rf / && true',
                    'rm -rf ~ && echo gone', 'rm -rf /etc; ls']:
            issues = _check_dangerous_commands(cmd)
            assert len(issues) >= 1, f"Should detect {cmd}"

    def test_subpaths_of_system_dirs_allowed(self):
        # Deleting under a system root is routine (e.g. Docker apt cleanup);
        # only removal of the root itself should be blocked.
        for cmd in ['rm -rf /var/lib/apt/lists/*',
                    'rm -rf /var/tmp/build_cache',
                    'rm -rf /usr/local/share/mystuff']:
            issues = _check_dangerous_commands(cmd)
            assert len(issues) == 0, f"Safe command flagged: {cmd}"

    def test_chmod_777_non_root_path_allowed(self):
        issues = _check_dangerous_commands('chmod -R 777 /home/user/my_ws')
        assert len(issues) == 0

    def test_chmod_777_root_in_compound_command(self):
        issues = _check_dangerous_commands('chmod -R 777 / && echo done')
        assert len(issues) >= 1

    def test_safe_commands_pass(self):
        safe_commands = [
            'colcon build',
            'ros2 run demo_nodes_cpp talker',
            'rm -rf build/ install/ log/',
            'cat /etc/os-release',
        ]
        for cmd in safe_commands:
            issues = _check_dangerous_commands(cmd)
            assert len(issues) == 0, f"Safe command flagged: {cmd}"

    def test_dangerous_patterns_non_empty(self):
        assert len(DANGEROUS_COMMAND_PATTERNS) >= 5


class TestPowerShellDangerousCommands:
    """PowerShell / Windows destructive-command coverage.

    Maintainer works on Windows and `pwsh`/`powershell` invocations can be
    forwarded under TOOL_NAME=Bash by the harness. The bash-only patterns left
    the entire Windows surface unguarded — Remove-Item, Format-Volume, etc.
    slipped through. These checks pin the regression and verify that safe
    PowerShell operations are not over-matched.
    """

    def test_remove_item_drive_root(self):
        issues = _check_dangerous_commands('Remove-Item -Recurse -Force C:/')
        assert len(issues) >= 1
        assert any('drive root' in i['message'].lower() for i in issues)

    def test_remove_item_drive_root_backslash(self):
        issues = _check_dangerous_commands('Remove-Item -Recurse -Force C:\\')
        assert len(issues) >= 1

    def test_remove_item_flag_order_swapped(self):
        # PowerShell parameter order is free — -Force first must also be caught.
        issues = _check_dangerous_commands('Remove-Item -Force -Recurse C:/')
        assert len(issues) >= 1

    def test_remove_item_case_insensitive(self):
        # PowerShell cmdlets are case-insensitive.
        issues = _check_dangerous_commands('remove-item -recurse -force c:/')
        assert len(issues) >= 1

    def test_remove_item_home(self):
        for target in ['$HOME', '$env:USERPROFILE', '~']:
            issues = _check_dangerous_commands(
                f'Remove-Item -Recurse -Force {target}')
            assert len(issues) >= 1, f'should flag home target {target!r}'

    def test_remove_item_windows_directories(self):
        for d in ['Windows', 'Program Files', 'Program Files (x86)', 'Users']:
            issues = _check_dangerous_commands(
                f'Remove-Item -Recurse -Force C:/{d}')
            assert len(issues) >= 1, f'should flag critical dir {d!r}'

    def test_format_volume(self):
        issues = _check_dangerous_commands('Format-Volume -DriveLetter C')
        assert len(issues) >= 1
        assert any('format' in i['message'].lower() for i in issues)

    def test_clear_disk(self):
        issues = _check_dangerous_commands('Clear-Disk -Number 0 -RemoveData')
        assert len(issues) >= 1
        assert any('clear' in i['message'].lower() for i in issues)

    def test_remove_partition(self):
        issues = _check_dangerous_commands(
            'Remove-Partition -DriveLetter D -Confirm:$false')
        assert len(issues) >= 1
        assert any('partition' in i['message'].lower() for i in issues)

    def test_rmdir_drive_root(self):
        issues = _check_dangerous_commands('rmdir /s /q C:\\')
        assert len(issues) >= 1

    def test_safe_powershell_commands_pass(self):
        safe = [
            'Get-Item C:/',
            'Remove-Item C:/Users/me/build',  # specific subdir, not root
            'Format-Table',                   # not Format-Volume
            'Get-ChildItem -Recurse -Force',  # no destructive verb
            'Clear-Host',                     # not Clear-Disk
            'New-Item -ItemType Directory C:/temp/build',
        ]
        for cmd in safe:
            issues = _check_dangerous_commands(cmd)
            assert len(issues) == 0, f'safe PS command flagged: {cmd!r}'


class TestPowerShellToolName:
    """The hook must route PowerShell tool invocations through the same
    dangerous-command pipeline. Previously the tool-name allowlist contained
    only bash-like aliases, so `TOOL_NAME=PowerShell` skipped the check
    entirely even when the payload was a destructive command.
    """

    def _run(self, tool_name, command):
        tool_input = json.dumps({'command': command})
        return subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_validate_hook.py')],
            capture_output=True, text=True,
            env={**os.environ,
                 'TOOL_NAME': tool_name, 'TOOL_INPUT': tool_input},
        )

    def test_powershell_destructive_blocked(self):
        result = self._run('PowerShell', 'Remove-Item -Recurse -Force C:/')
        assert result.returncode == 1, \
            'PowerShell tool name must route to dangerous-command check'
        data = json.loads(result.stdout)
        assert data['status'] == 'fail'

    def test_pwsh_destructive_blocked(self):
        result = self._run('pwsh', 'Format-Volume -DriveLetter C')
        assert result.returncode == 1
        data = json.loads(result.stdout)
        assert data['status'] == 'fail'

    def test_cmd_destructive_blocked(self):
        result = self._run('cmd', 'rmdir /s /q C:\\')
        assert result.returncode == 1
        data = json.loads(result.stdout)
        assert data['status'] == 'fail'

    def test_bash_destructive_still_blocked(self):
        # Regression guard: PowerShell additions must not have weakened the
        # existing bash branch.
        result = self._run('Bash', 'rm -rf /')
        assert result.returncode == 1
        data = json.loads(result.stdout)
        assert data['status'] == 'fail'

    def test_rm_rf_root_cli(self):
        """Test via CLI that rm -rf / is blocked."""
        tool_input = json.dumps({'command': 'rm -rf /'})
        result = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_validate_hook.py')],
            capture_output=True, text=True,
            env={**os.environ,
                 'TOOL_NAME': 'Bash', 'TOOL_INPUT': tool_input},
        )
        assert result.returncode == 1
        data = json.loads(result.stdout)
        assert data['status'] == 'fail'


class TestValidateHookCLI:
    """Test the PreToolUse hook as a CLI command."""

    def test_no_input_passes(self):
        result = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_validate_hook.py')],
            capture_output=True, text=True,
            env={**os.environ,
                 'TOOL_NAME': '', 'TOOL_INPUT': ''},
        )
        assert result.returncode == 0
        data = json.loads(result.stdout)
        assert data['status'] == 'pass'

    def test_write_clean_code_passes(self):
        tool_input = json.dumps({
            'file_path': 'test.py',
            'content': 'import rclpy\nclass MyNode: pass\n',
        })
        result = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_validate_hook.py')],
            capture_output=True, text=True,
            env={**os.environ,
                 'TOOL_NAME': 'Write', 'TOOL_INPUT': tool_input},
        )
        assert result.returncode == 0
        data = json.loads(result.stdout)
        assert data['status'] == 'pass'

    def test_write_antipattern_warns(self):
        tool_input = json.dumps({
            'file_path': 'test.py',
            'content': 'time.sleep(5)\n',
        })
        result = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_validate_hook.py')],
            capture_output=True, text=True,
            env={**os.environ,
                 'TOOL_NAME': 'Write', 'TOOL_INPUT': tool_input},
        )
        assert result.returncode == 0  # Warnings don't block
        data = json.loads(result.stdout)
        assert data['issues_count'] >= 1

    def test_dangerous_bash_command_fails(self):
        tool_input = json.dumps({
            'command': 'rm -rf /opt/ros',
        })
        result = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_validate_hook.py')],
            capture_output=True, text=True,
            env={**os.environ,
                 'TOOL_NAME': 'Bash', 'TOOL_INPUT': tool_input},
        )
        assert result.returncode == 1
        data = json.loads(result.stdout)
        assert data['status'] == 'fail'

    def test_output_is_valid_json(self):
        result = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_validate_hook.py')],
            capture_output=True, text=True,
            env={**os.environ,
                 'TOOL_NAME': '', 'TOOL_INPUT': ''},
        )
        data = json.loads(result.stdout)
        assert 'hook' in data
        assert 'version' in data
        assert 'issues_count' in data
        assert 'issues' in data
        assert 'status' in data

    def test_edit_tool_with_antipattern(self):
        tool_input = json.dumps({
            'file_path': 'test.py',
            'new_string': 'global node_state\n',
        })
        result = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_validate_hook.py')],
            capture_output=True, text=True,
            env={**os.environ,
                 'TOOL_NAME': 'Edit', 'TOOL_INPUT': tool_input},
        )
        assert result.returncode == 0  # Warnings don't block
        data = json.loads(result.stdout)
        assert data['issues_count'] >= 1

    def test_invalid_json_input_passes(self):
        result = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_validate_hook.py')],
            capture_output=True, text=True,
            env={**os.environ,
                 'TOOL_NAME': 'Write', 'TOOL_INPUT': 'not json'},
        )
        assert result.returncode == 0
        data = json.loads(result.stdout)
        assert data['status'] == 'pass'


class TestStopHookMainDirect:
    """Test skill_stop_hook.main() directly for coverage."""

    def test_main_clean_workspace(self, tmp_path, monkeypatch):
        import pytest as _pytest
        from skill_stop_hook import main
        monkeypatch.setenv('SKILL_WORKSPACE', str(tmp_path))
        with _pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 0

    def test_main_with_valid_launch(self, tmp_path, monkeypatch):
        import pytest as _pytest
        from skill_stop_hook import main
        (tmp_path / 'launch').mkdir()
        (tmp_path / 'launch' / 'ok.launch.py').write_text(
            'from launch import LaunchDescription\n'
            'def generate_launch_description():\n'
            '    return LaunchDescription([])\n'
        )
        monkeypatch.setenv('SKILL_WORKSPACE', str(tmp_path))
        with _pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 0

    def test_main_with_error_launch(self, tmp_path, monkeypatch):
        import pytest as _pytest
        from skill_stop_hook import main
        (tmp_path / 'launch').mkdir()
        (tmp_path / 'launch' / 'bad.launch.py').write_text(
            'from launch import LaunchDescription\n'
            'def wrong_name():\n'
            '    return LaunchDescription([])\n'
        )
        monkeypatch.setenv('SKILL_WORKSPACE', str(tmp_path))
        with _pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 1

    def test_main_with_package_xml(self, tmp_path, monkeypatch):
        import pytest as _pytest
        from skill_stop_hook import main
        (tmp_path / 'package.xml').write_text(
            '<?xml version="1.0"?>\n'
            '<package format="3"><name>t</name>'
            '<license>Apache-2.0</license></package>\n'
        )
        monkeypatch.setenv('SKILL_WORKSPACE', str(tmp_path))
        with _pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 0

    def test_main_missing_name_in_pkg_xml(self, tmp_path, monkeypatch):
        import pytest as _pytest
        from skill_stop_hook import main
        (tmp_path / 'package.xml').write_text(
            '<?xml version="1.0"?>\n'
            '<package format="3"><license>Apache-2.0</license></package>\n'
        )
        monkeypatch.setenv('SKILL_WORKSPACE', str(tmp_path))
        with _pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 1


class TestValidateHookMainDirect:
    """Test skill_validate_hook.main() directly for coverage."""

    def test_main_no_input(self, monkeypatch):
        import pytest as _pytest
        from skill_validate_hook import main
        monkeypatch.setenv('TOOL_NAME', '')
        monkeypatch.setenv('TOOL_INPUT', '')
        with _pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 0

    def test_main_write_clean(self, monkeypatch):
        import pytest as _pytest
        from skill_validate_hook import main
        monkeypatch.setenv('TOOL_NAME', 'Write')
        monkeypatch.setenv('TOOL_INPUT', json.dumps({
            'file_path': 'test.py',
            'content': 'import rclpy\n',
        }))
        with _pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 0

    def test_main_write_antipattern(self, monkeypatch):
        import pytest as _pytest
        from skill_validate_hook import main
        monkeypatch.setenv('TOOL_NAME', 'Write')
        monkeypatch.setenv('TOOL_INPUT', json.dumps({
            'file_path': 'test.py',
            'content': 'time.sleep(5)\n',
        }))
        with _pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 0  # Warnings don't block

    def test_main_edit_antipattern(self, monkeypatch):
        import pytest as _pytest
        from skill_validate_hook import main
        monkeypatch.setenv('TOOL_NAME', 'Edit')
        monkeypatch.setenv('TOOL_INPUT', json.dumps({
            'file_path': 'test.py',
            'new_string': 'global node_ref\n',
        }))
        with _pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 0

    def test_main_bash_dangerous(self, monkeypatch):
        import pytest as _pytest
        from skill_validate_hook import main
        monkeypatch.setenv('TOOL_NAME', 'Bash')
        monkeypatch.setenv('TOOL_INPUT', json.dumps({
            'command': 'rm -rf /opt/ros',
        }))
        with _pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 1

    def test_main_bash_safe(self, monkeypatch):
        import pytest as _pytest
        from skill_validate_hook import main
        monkeypatch.setenv('TOOL_NAME', 'Bash')
        monkeypatch.setenv('TOOL_INPUT', json.dumps({
            'command': 'colcon build',
        }))
        with _pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 0

    def test_main_bash_invalid_json(self, monkeypatch):
        import pytest as _pytest
        from skill_validate_hook import main
        monkeypatch.setenv('TOOL_NAME', 'Bash')
        monkeypatch.setenv('TOOL_INPUT', 'not json')
        with _pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 0

    def test_main_write_no_content(self, monkeypatch):
        import pytest as _pytest
        from skill_validate_hook import main
        monkeypatch.setenv('TOOL_NAME', 'Write')
        monkeypatch.setenv('TOOL_INPUT', json.dumps({
            'file_path': 'test.py',
        }))
        with _pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 0

    def test_main_invalid_json_write(self, monkeypatch):
        import pytest as _pytest
        from skill_validate_hook import main
        monkeypatch.setenv('TOOL_NAME', 'Write')
        monkeypatch.setenv('TOOL_INPUT', '{bad json')
        with _pytest.raises(SystemExit) as exc_info:
            main()
        assert exc_info.value.code == 0


class TestClaudeCodeStdinPayload:
    """Real Claude Code sends the hook payload as JSON on STDIN, not via
    env vars. Schema verified 2026-05-21 by capturing actual fires::

        {"session_id":..., "cwd":..., "hook_event_name":"PreToolUse",
         "tool_name":"Bash", "tool_input": {"command": "..."}, ...}

    Note `tool_input` is already a dict, not a JSON string.
    """

    def _run_stdin(self, payload):
        return subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_validate_hook.py')],
            input=json.dumps(payload),
            capture_output=True, text=True,
            # Strip env vars that could short-circuit the stdin path
            env={k: v for k, v in os.environ.items()
                 if k not in ('TOOL_NAME', 'TOOL_INPUT')},
        )

    def test_stdin_bash_destructive_blocks(self):
        r = self._run_stdin({
            'session_id': 'test', 'cwd': '/tmp',
            'hook_event_name': 'PreToolUse',
            'tool_name': 'Bash',
            'tool_input': {'command': 'rm -rf /'},
        })
        assert r.returncode == 1
        data = json.loads(r.stdout)
        assert data['status'] == 'fail'
        assert data['issues_count'] >= 1

    def test_stdin_powershell_destructive_blocks(self):
        r = self._run_stdin({
            'session_id': 'test', 'cwd': '/tmp',
            'hook_event_name': 'PreToolUse',
            'tool_name': 'Bash',  # Claude Code surfaces shell calls as Bash
            'tool_input': {'command': 'Remove-Item -Recurse -Force C:/'},
        })
        assert r.returncode == 1

    def test_stdin_edit_antipattern_warns_passes(self):
        r = self._run_stdin({
            'session_id': 'test', 'cwd': '/tmp',
            'hook_event_name': 'PreToolUse',
            'tool_name': 'Edit',
            'tool_input': {
                'file_path': '/tmp/x.py',
                'old_string': 'pass',
                'new_string': 'import time\ntime.sleep(1)',
            },
        })
        assert r.returncode == 0  # warning, not blocking
        data = json.loads(r.stdout)
        assert data['issues_count'] == 1
        assert 'time.sleep' in data['issues'][0]['message']

    def test_stdin_multiedit_flattens_all_edits(self):
        r = self._run_stdin({
            'session_id': 'test', 'cwd': '/tmp',
            'hook_event_name': 'PreToolUse',
            'tool_name': 'MultiEdit',
            'tool_input': {
                'file_path': '/tmp/x.py',
                'edits': [
                    {'old_string': 'a', 'new_string': 'clean = 1'},
                    {'old_string': 'b', 'new_string': 'time.sleep(2)'},
                ],
            },
        })
        # MultiEdit must scan the concatenated new_strings for antipatterns.
        data = json.loads(r.stdout)
        assert any('time.sleep' in i['message']
                   for i in data['issues']), data

    def test_stdin_tool_input_as_dict_not_string(self):
        # Real Claude Code sends tool_input as object, not a JSON string.
        # Hook must NOT try to json.loads it a second time.
        r = self._run_stdin({
            'tool_name': 'Write',
            'tool_input': {
                'file_path': '/tmp/x.py',
                'content': 'def main(): pass\n',
            },
        })
        assert r.returncode == 0
        data = json.loads(r.stdout)
        assert data['status'] == 'pass'

    def test_stdin_empty_input_does_not_crash(self):
        r = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_validate_hook.py')],
            input='',
            capture_output=True, text=True,
            env={k: v for k, v in os.environ.items()
                 if k not in ('TOOL_NAME', 'TOOL_INPUT')},
        )
        assert r.returncode == 0
        data = json.loads(r.stdout)
        assert data['status'] == 'pass'
        assert data['issues_count'] == 0

    def test_stdin_malformed_json_falls_back_safely(self):
        r = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_validate_hook.py')],
            input='{not valid json',
            capture_output=True, text=True,
            env={k: v for k, v in os.environ.items()
                 if k not in ('TOOL_NAME', 'TOOL_INPUT')},
        )
        # Should not crash; falls through to env (which we cleared) so no
        # tool_name -> no checks run -> pass.
        assert r.returncode == 0

    def test_stdin_takes_precedence_over_env_for_real_payload(self):
        r = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_validate_hook.py')],
            input=json.dumps({
                'tool_name': 'Bash',
                'tool_input': {'command': 'rm -rf /'},
            }),
            capture_output=True, text=True,
            env={**os.environ,
                 'TOOL_NAME': 'Write',  # would be safe
                 'TOOL_INPUT': '{"file_path":"/tmp/x.py","content":"clean"}'},
        )
        # Stdin has destructive Bash -> must block, even though env says Write.
        assert r.returncode == 1

    def test_debug_mode_emits_debug_info(self):
        r = subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_validate_hook.py'),
             '--debug'],
            input=json.dumps({
                'tool_name': 'Bash',
                'tool_input': {'command': 'colcon build'},
            }),
            capture_output=True, text=True,
            env={k: v for k, v in os.environ.items()
                 if k not in ('TOOL_NAME', 'TOOL_INPUT')},
        )
        data = json.loads(r.stdout)
        assert 'debug' in data
        assert data['debug']['source'] == 'stdin'
        assert data['debug']['tool_name'] == 'Bash'


class TestStopHookWorkspaceResolution:
    """Stop hook must pick the workspace from real Claude Code's stdin
    payload (cwd field) or CLAUDE_PROJECT_DIR env, not just cwd().
    """

    def _run_stop(self, payload=None, env_overrides=None, cwd=None):
        env = {k: v for k, v in os.environ.items()
               if k not in ('SKILL_WORKSPACE', 'CLAUDE_PROJECT_DIR')}
        if env_overrides:
            env.update(env_overrides)
        return subprocess.run(
            [sys.executable,
             os.path.join(SCRIPTS_DIR, 'skill_stop_hook.py')],
            input=json.dumps(payload) if payload else '',
            capture_output=True, text=True,
            env=env, cwd=cwd,
        )

    def test_stdin_cwd_used_when_no_env(self, tmp_path):
        # Create a fake pkg.xml in tmp_path; stop hook should find it
        # because it walks the cwd from stdin payload.
        pkg = tmp_path / 'package.xml'
        pkg.write_text(
            '<?xml version="1.0"?><package format="3">'
            '<name>x</name><license>Apache-2.0</license>'
            '</package>',
            encoding='utf-8')
        r = self._run_stop({'cwd': str(tmp_path),
                            'hook_event_name': 'Stop'})
        data = json.loads(r.stdout)
        # Pass means it scanned + found a clean package.xml.
        assert data['status'] == 'pass'
        assert data['issues_count'] == 0

    def test_claude_project_dir_env_used_when_no_stdin(self, tmp_path):
        # No stdin payload, but CLAUDE_PROJECT_DIR env -> use that.
        pkg = tmp_path / 'package.xml'
        pkg.write_text(
            '<?xml version="1.0"?><package format="3">'
            '<name>x</name><license>Apache-2.0</license>'
            '</package>',
            encoding='utf-8')
        r = self._run_stop(
            env_overrides={'CLAUDE_PROJECT_DIR': str(tmp_path)},
        )
        data = json.loads(r.stdout)
        assert data['status'] == 'pass'

    def test_skill_workspace_env_overrides_everything(self, tmp_path):
        # Even if stdin payload says a different cwd, SKILL_WORKSPACE wins
        # (used by pytest for hermetic test workspaces).
        other = tmp_path / 'other'
        other.mkdir()
        target = tmp_path / 'target'
        target.mkdir()
        r = self._run_stop(
            payload={'cwd': str(other), 'hook_event_name': 'Stop'},
            env_overrides={'SKILL_WORKSPACE': str(target)},
        )
        # Both dirs are empty (no package.xml/launch) -> clean pass.
        assert r.returncode == 0


class TestReadToolContextDirect:
    """Cover _read_tool_context branches directly so we can assert the
    parsing precisely rather than only through CLI surface."""

    def test_env_fallback_with_invalid_json(self, monkeypatch):
        # Reading the source module directly is required so monkeypatch on
        # sys.stdin and os.environ takes effect inside the call.
        sys.path.insert(0, SCRIPTS_DIR)
        from skill_validate_hook import _read_tool_context
        import io
        # Empty stdin -> falls through to env
        monkeypatch.setattr('sys.stdin', io.StringIO(''))
        monkeypatch.setenv('TOOL_NAME', 'Bash')
        monkeypatch.setenv('TOOL_INPUT', '{not valid json')
        name, data, debug = _read_tool_context()
        assert name == 'Bash'
        assert data == {}  # malformed env JSON -> empty dict
        assert debug['source'] == 'env'
        assert 'env_parse_error' in debug

    def test_stdin_non_dict_payload_falls_through(self, monkeypatch):
        sys.path.insert(0, SCRIPTS_DIR)
        from skill_validate_hook import _read_tool_context
        import io
        # Top-level JSON array (not dict) -> ignored, fall through to env
        monkeypatch.setattr('sys.stdin', io.StringIO('[1, 2, 3]'))
        monkeypatch.delenv('TOOL_NAME', raising=False)
        monkeypatch.delenv('TOOL_INPUT', raising=False)
        name, data, debug = _read_tool_context()
        assert name == ''
        assert data == {}
        # Source falls through to env since stdin payload was unusable.
        assert debug['source'] == 'env'

    def test_stdin_tool_input_non_dict_normalized_to_empty(self, monkeypatch):
        # Real Claude Code always sends tool_input as object, but defensively
        # if a future schema version wraps it (e.g. as string), we must not
        # crash - we normalize to empty dict.
        sys.path.insert(0, SCRIPTS_DIR)
        from skill_validate_hook import _read_tool_context
        import io
        monkeypatch.setattr('sys.stdin',
                            io.StringIO(json.dumps({
                                'tool_name': 'Bash',
                                'tool_input': 'should-be-an-object-not-string',
                            })))
        monkeypatch.delenv('TOOL_NAME', raising=False)
        name, data, debug = _read_tool_context()
        assert name == 'Bash'
        assert data == {}  # non-dict tool_input normalized
        assert debug['source'] == 'stdin'


class TestResolveWorkspaceDirect:
    """Cover _resolve_workspace branches directly."""

    def test_falls_back_to_cwd_when_nothing_else(self, monkeypatch, tmp_path):
        sys.path.insert(0, SCRIPTS_DIR)
        from skill_stop_hook import _resolve_workspace
        import io
        monkeypatch.delenv('SKILL_WORKSPACE', raising=False)
        monkeypatch.delenv('CLAUDE_PROJECT_DIR', raising=False)
        monkeypatch.setattr('sys.stdin', io.StringIO(''))
        monkeypatch.chdir(str(tmp_path))
        ws = _resolve_workspace()
        # Resolve via realpath comparison (tmp_path on Windows may have
        # different drive-letter casing than os.getcwd()).
        assert os.path.realpath(ws) == os.path.realpath(str(tmp_path))

    def test_stdin_cwd_ignored_if_not_a_directory(self, monkeypatch, tmp_path):
        sys.path.insert(0, SCRIPTS_DIR)
        from skill_stop_hook import _resolve_workspace
        import io
        monkeypatch.delenv('SKILL_WORKSPACE', raising=False)
        monkeypatch.delenv('CLAUDE_PROJECT_DIR', raising=False)
        # Path that does not exist -> falls through.
        monkeypatch.setattr('sys.stdin', io.StringIO(json.dumps({
            'cwd': r'C:\definitely\not\a\real\directory\xyz',
        })))
        monkeypatch.chdir(str(tmp_path))
        ws = _resolve_workspace()
        # Did not honor the bogus cwd; fell through to os.getcwd().
        assert os.path.realpath(ws) == os.path.realpath(str(tmp_path))

    def test_claude_project_dir_used_when_no_stdin(self, monkeypatch, tmp_path):
        sys.path.insert(0, SCRIPTS_DIR)
        from skill_stop_hook import _resolve_workspace
        import io
        monkeypatch.delenv('SKILL_WORKSPACE', raising=False)
        monkeypatch.setenv('CLAUDE_PROJECT_DIR', str(tmp_path))
        monkeypatch.setattr('sys.stdin', io.StringIO(''))
        assert os.path.realpath(_resolve_workspace()) == os.path.realpath(
            str(tmp_path))
