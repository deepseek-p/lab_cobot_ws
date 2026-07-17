# Repository Guidelines

## Project Structure & Module Organization

This ROS 2 Humble colcon workspace targets Gazebo Classic, Nav2, and MoveIt 2. First-party packages live under `src/lab_cobot_*`: `description` owns URDF/SRDF, `gazebo` owns worlds and C++ plugins, `navigation` owns Nav2/maps, `moveit` owns planning configuration, `perception` owns camera/ArUco processing, `manipulation` owns pick/place logic, and `bringup` owns the integrated launch and `mission_node` state machine. Tests live in each package's `test/`. Supporting material belongs in `docs/`, `benchmarks/`, and `tools/`. Never modify vendored `src/pymoveit2/`.

## Build, Test, and Development Commands

Source ROS before working:

```bash
source /opt/ros/humble/setup.bash
export PYTEST_ADDOPTS='-p no:anyio'
colcon build --symlink-install --packages-select <pkg>
source install/setup.bash
colcon test --packages-select <pkg> --event-handlers console_direct+
colcon test-result --verbose
```

Run one test with `python3 -m pytest src/<pkg>/test/test_name.py -p no:anyio -q`. Start simulation with `ros2 launch lab_cobot_bringup lab_cobot.launch.py`; add `gui:=false use_rviz:=false` for headless operation. Validate maps with `python3 src/lab_cobot_navigation/maps/check_map.py`.

## Coding Style & Naming Conventions

Use four-space indentation and `snake_case` for Python modules, functions, parameters, and tests. Keep new docstrings single-line English sentences ending with an ASCII period; retain existing Chinese comments and documentation. Keep Python lines within 99 characters. Follow existing ROS 2/CMake patterns. Format new C++ through `ament_uncrustify --reformat <files>` instead of hand-tuning style.

## Testing Guidelines

Python tests use pytest/ament lint; Gazebo plugin logic also uses gtest. Name files `test_*.py` or `test_*.cpp`. Do not add `ament_add_pytest_test` to the ament_python `manipulation` or `perception` packages; colcon already discovers their tests. Launch tests must inspect generated actions, never source-code substrings. Tests must remain offline, CPU-safe, and free of model downloads. Register tests longer than 60 seconds separately. Before E2E/SIM, clear Gazebo and `/opt/ros/humble/lib` processes as documented in `CLAUDE.md`; never use a self-matching `pkill -f 'gzserver|gzclient'` command.

## Commit & Pull Request Guidelines

Use `<type>: <中文简述>` with `feat`, `fix`, `refactor`, `docs`, `test`, or `chore`. Use body bullets for rationale and verified evidence; do not add attribution trailers. Keep commits scoped and green. PRs should identify affected packages, link the issue/goal, list exact build/test results, and include commands plus key output or screenshots for simulation-visible changes.

## Remote Push Policy

Never run `git push origin main` directly: local history contains internal documents. The remote `main` only accepts sanitized snapshot commits parented on `origin/main`, excluding `.claude/`, `.codex/`, `docs/`, `CLAUDE.md`, and `AGENTS.md`, and keeping `src/`, `tools/`, `benchmarks/`, `README.md`, `THIRD_PARTY_LICENSES.md`, and `.gitignore` (the license file covers vendored `pymoveit2` and must stay). Use the recipe in `CLAUDE.md` ("远端推送标准"): build a temporary index with `GIT_INDEX_FILE`, `git rm -r --cached` the excluded paths, `git write-tree`, `git commit-tree -p origin/main`, then push the resulting commit to `refs/heads/main` (fast-forward, no force). Verify afterwards with `git ls-tree --name-only <commit>` that only the six kept entries remain at top level.

## Safety & Configuration

Read `docs/MVP-SPEC-for-codex.md`, `docs/STATUS_HONEST.md`, and `CLAUDE.md` before changing launch behavior. Preserve locked defaults, maps/provenance, grasp safety values, and honest-E2E assertions; `use_tactile_grasp` and `require_finger_contact` must remain paired. Launch/lifecycle changes require a real `gui:=true` start/stop check in addition to headless tests. Do not claim unverified physical realism. Never commit model weights, API keys, `.env` files, or generated `build/`, `install/`, and `log/` trees.
