# Repository Guidelines

This repository is a ROS 1 (Melodic, catkin) C++17 package for LiDAR SLAM and localization (`project(lightning)`). Use this guide to navigate, build, and contribute efficiently.

## Project Structure & Module Organization
- `src/` core C++ and apps: library `${PROJECT_NAME}.libs` and executables in `src/app/` (e.g., `run_slam_offline`, `run_loc_online`).
- `srv/` service definitions (catkin `message_generation`).
- `config/` YAML configs (e.g., `default_nclt.yaml`).
- `scripts/` helper scripts (e.g., `scripts/install_dep.sh`).
- `thirdparty/` vendored deps; includes `livox_ros_driver` (ROS1 messages).
- Build system: `CMakeLists.txt`, `cmake/packages.cmake` (dependency discovery).
- Assets/docs: `doc/`, example data in `pcd/`.

## Build, Test, and Development Commands
- Workspace: place this repo under a catkin workspace (`catkin_ws/src/`).
- Build: `catkin_make` then `source devel/setup.bash`.
- Run (offline): `rosrun lightning run_slam_offline --config ./config/default_nclt.yaml --input_bag /path/to.bag`.
- Run (online): `rosrun lightning run_slam_online --config ./config/default_nclt.yaml`.
- Tests: if adding gtests, use `catkin_add_gtest` in CMake and run `catkin_make run_tests`.

## Coding Style & Naming Conventions
- C++17; 4-space indent; 120-col limit. Enforced by `.clang-format` (Google-based). Example: `clang-format -i src/**/*.cc`.
- Filenames: snake_case (`.cc`, `.h`). Types: PascalCase. Functions/variables: lower_snake_case. Constants/macros: UPPER_SNAKE_CASE.
- Keep includes project-relative (under `src/`) and minimize headers in headers.

## Testing Guidelines
- Add unit tests for core logic (gtest + catkin). Prefer small, deterministic tests.
- Provide a smoke path: offline command produces `data/<map_id>/global.pcd` and (if enabled) `map.pgm/yaml`.

## Commit & Pull Request Guidelines
- Commits: concise, imperative summary (<=72 chars), details in body, reference issues (e.g., `Fixes #123`).
- PRs: describe what/why, run/verify steps (commands), and attach visuals for UI/visual outputs. Update `README.md`/`config/` when behavior changes.

## Security & Configuration Tips
- Do not commit large datasets, bag files, or generated maps. Paths like `build/`, `devel/`, `log/`, and `data/` should stay out of VCS.
- Keep secrets/config paths out of code; use YAML configs.
