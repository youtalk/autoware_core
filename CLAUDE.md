# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

`autoware_core` is the [Autoware](https://github.com/autowarefoundation/autoware) **Core** module: a curated set of stable, high-quality ROS 2 (Jazzy) packages for autonomous driving. It is the conservative counterpart to [Autoware Universe](https://github.com/autowarefoundation/autoware_universe), which holds experimental packages. When porting or comparing code, expect a Universe equivalent to exist; Core favors stability, well-defined interfaces, and test coverage over cutting-edge features.

This checkout is one ROS package source tree inside a larger colcon workspace (it lives at `<ws>/src/core/autoware_core`). It is **not** built in place — see Build.

## Fork / upstream workflow (read before committing or opening PRs)

This clone has two remotes:

- `origin` → `git@github.com:youtalk/autoware_core.git` — the personal fork.
- `upstream` → `git@github.com:autowarefoundation/autoware_core.git` — the canonical Autoware Core repo. Local `main` tracks `upstream/main`.

Two categories of change are **fork-local only** and live on `origin/main` (or fork branches), but must **never** appear in a branch or PR that targets `upstream`:

1. `CLAUDE.md` and any other Claude/agent-tooling files.
2. Ubicloud-related fixes (self-hosted CI runner / infrastructure adjustments for the fork's GitHub Actions).

Consequences for day-to-day work:

- Cut every branch intended for an **upstream PR** from `upstream/main` (e.g. `git fetch upstream && git switch -c feat/<x> upstream/main`), **not** from the fork's `main` — otherwise the fork-local commits ride along.
- Upstream PR branches must contain only the feature/fix commits. If fork-local commits have leaked in, scrub them with `git rebase --onto upstream/main` or interactive rebase before pushing the PR branch.
- Fork-local commits (CLAUDE.md, Ubicloud) are committed and pushed to `origin/main` so they persist across machines, but are excluded from anything sent to `autowarefoundation`.

## Build, test, lint

Native colcon builds run in the `~/ros/jazzy` ROS workspace (overlay on `/opt/ros/jazzy`). Always source the environment first:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros/jazzy
source install/setup.bash   # once install/ exists
```

```bash
# Build one package and its dependencies (preferred while iterating)
colcon build --symlink-install --packages-up-to <pkg> --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build only one package (deps already built)
colcon build --symlink-install --packages-select <pkg>

# Test one package, then view results
colcon test --packages-select <pkg> --event-handlers console_cohesion+
colcon test-result --verbose

# Run a single gtest case (ctest filters by test name registered in CMake)
colcon test --packages-select <pkg> --ctest-args -R <test_name>
# or run the built binary directly:
~/ros/jazzy/build/<pkg>/<test_binary> --gtest_filter='Suite.Case'
```

`<pkg>` is the `package.xml` `<name>` (e.g. `autoware_trajectory`, `autoware_ekf_localizer`), not the directory.

Lint / formatting is enforced by pre-commit and CI, not by the build:

```bash
pre-commit run -a            # all hooks (clang-format, cpplint, markdownlint, prettier, black, isort, yamllint, shellcheck, shfmt, sort-package-xml, hadolint, cspell, ...)
pre-commit run clang-format --all-files
```

## Conventions that the CI will reject if violated

- **Pull request titles must be Conventional Commits** (`semantic-pull-request.yaml` workflow). The scope is conventionally the affected package, e.g. `fix(autoware_core_localization): ...`, `feat(autoware_agnocast_wrapper): ...`.
- **C++ is C++17**, Google-based clang-format with project overrides: `ColumnLimit: 100`, `PointerAlignment: Middle`, braces wrapped after class/function/namespace/struct (see `.clang-format`).
- **clang-tidy** runs on PRs (`build-test-tidy-pr.yaml`, posts inline comments). A large check set is enabled; `.clang-tidy` `WarningsAsErrors` lists the subset that fails the build. Identifier naming is enforced: namespaces/functions/variables `lower_case`, classes/structs `CamelCase`, private members suffixed `_`, global constants prefixed `g_`.
- **cppcheck** and **spell-check (cspell)** run differentially on PRs and daily. Add legitimate domain words to `.cspell.json`; suppress false cppcheck positives in `.cppcheck_suppressions`.
- **`package.xml` must stay sorted** (`sort-package-xml` hook) and dependency hygiene is checked (`check-package-depends`).
- JSON parameter schemas under any `**/schema/*.schema.json` are validated (`json-schema-check.yaml`).
- Per-package versions live in `package.xml` and `CHANGELOG.rst`; version bumps are automated (`bump-version-pr.yaml`, `release-new-version-when-merged.yaml`) — do not hand-edit versions in normal PRs.

## Package layout and the meta-package pattern

~71 packages are grouped by domain directory: `common/`, `control/`, `localization/`, `map/`, `perception/`, `planning/`, `sensing/`, `vehicle/`, `api/`, `testing/`, `description/`.

Each domain has a thin **meta package** named `autoware_core_<domain>` (`autoware_core_control`, `autoware_core_localization`, `autoware_core_map`, `autoware_core_perception`, `autoware_core_planning`, `autoware_core_sensing`, `autoware_core_vehicle`) plus the top-level `autoware_core`. These contain no algorithms — they aggregate runtime dependencies and ship the integrated launch files / parameter configs that wire the domain's packages together. When a parameter needs to be exposed end-to-end, it usually has to be threaded through both the implementing package and its `autoware_core_*` config (see commit `fix(autoware_core_localization): sync ekf_localizer param`).

A typical functional package is `ament_cmake_auto` based and starts its `CMakeLists.txt` with:

```cmake
find_package(autoware_cmake REQUIRED)
autoware_package()          # macro from autoware_cmake: sets C++ standard, deps, lint, exports
ament_auto_add_library(${PROJECT_NAME} SHARED src/...)
```

`autoware_package()` and `ament_auto_*` pull dependencies straight from `package.xml`, so **add dependencies to `package.xml`, not by hand-listing in CMake**. Tests use `ament_auto_add_gtest` / launch_test and depend on `autoware_lint_common`.

## Cross-cutting building blocks (read these before touching multiple packages)

- `common/autoware_component_interface_specs` — canonical topic/service interface definitions shared across the stack. Interface changes here ripple widely.
- `common/autoware_node` — the base node class most Core nodes derive from.
- `common/autoware_agnocast_wrapper` — abstraction over [agnocast](https://github.com/tier4/agnocast) zero-copy IPC; provides Timer / message_filter / diagnostic_updater wrappers so node code stays agnostic of the underlying transport.
- `common/` also holds the math/geometry/utility libraries (`autoware_trajectory`, `autoware_interpolation`, `autoware_motion_utils`, `autoware_lanelet2_utils`, `autoware_kalman_filter`, `autoware_osqp_interface` / `autoware_qp_interface`, `autoware_vehicle_info_utils`, ...). Prefer reusing these over re-implementing.
- `testing/` provides shared test infrastructure: `autoware_test_utils`, `autoware_planning_test_manager`, `autoware_test_node`, `autoware_pyplot` (C++ matplotlib bindings for test visualization), and `autoware_testing` (Python).

## Docs

Per-package `README.md` (and `schema/`, `config/`) are the source of truth for a package's behavior and parameters. The site is built with MkDocs (`mkdocs.yaml`, `mkdocs_macros.py`) and deployed by `deploy-docs.yaml`; PRs get a preview and closed-PR docs are cleaned up automatically.
