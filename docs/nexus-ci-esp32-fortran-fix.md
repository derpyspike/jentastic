# Nexus CI ESP32 Build Fix: SCons FortranCommon & Python 3.15 Resolution

**Author:** Jirachi ⭐ (`Noctali.local`) & Jenny 💖  
**Date:** 2026-10-20  
**Branch:** `fix/nexus-ci-fortran-esp32`  
**Affects:** `.github/workflows/nexus_build_firmware.yml`, `.github/workflows/nexus_main_matrix.yml`, `.github/workflows/nexus_draft_release.yml`, `.github/workflows/nexus_build_one_target.yml`, `bin/build-esp32.sh`

---

## 1. Problem Statement & Symptoms

In GitHub Actions run `106548712280` (`nexus_draft_release.yml`), all ESP32 architecture targets (`esp32`, `esp32s3`, `esp32c3`, `esp32c6`) failed completely. As a result, the draft release only contained firmware archives for `nrf52840`, `rp2040`, and `rp2350`.

The failures manifested in two distinct ways:

### Symptom A: Python 3.15 Resolution Failure (e.g. `heltec-v4`, `tlora-c6`)
```text
ERROR: Python version must be 3.10, 3.11, 3.12, 3.13, 3.14.
Current Python version: 3.15.0
Process completed with exit code 1.
```

### Symptom B: Fortran Linker Crash (e.g. `heltec_capsule_sensor_v3`, `tlora-v2`)
```text
*** [.pio/build/heltec_capsule_sensor_v3/firmware-heltec_capsule_sensor_v3-2.8.104.417e407.elf] ModuleNotFoundError : No module named 'SCons.Tool.FortranCommon'
========================= [FAILED] Took 35.30 seconds =========================
```

---

## 2. Root Cause Analysis

### Cause 1: `python-version: 3.x` resolves to 3.15.0
On GitHub Actions Ubuntu runners, `actions/setup-python@v7` with `python-version: 3.x` picks the highest available Python 3 version. With Python 3.15 available on runner toolcaches, `3.15.0` was selected. `pioarduino-platform-espressif32` validates Python version and rejects `>=3.15`.

### Cause 2: On-The-Fly penv Upgrade to `pioarduino 6.2.0` and SCons Wipe
1. The CI workflow ran `pip install -U platformio==6.1.19` in the runner's top-level environment.
2. `bin/build-esp32.sh` invoked `pio run`.
3. `pioarduino-platform-espressif32` initialized its own dedicated virtualenv at `~/.platformio/penv`.
4. In `penv_setup.py`, the dependency specification is `"pioarduino": ">=6.1.19"`.
5. Because PyPI published `pioarduino 6.2.0`, `uv pip install --upgrade pioarduino>=6.1.19` installed **`pioarduino 6.2.0`** into `~/.platformio/penv/bin/pio`.
6. ESP-IDF compilation stage 1 finished. Then `espidf.py` launched stage 2 (Arduino application compilation & link) by spawning child:
   `~/.platformio/penv/bin/pio run -e ...`
7. When `penv/bin/pio` (6.2.0) ran, `platformio.maintenance.after_upgrade()` detected that `appstate.json` had `"last_version": "6.1.19"` while the running binary was `6.2.0`.
8. `after_upgrade()` printed:
   ```text
   Please wait while upgrading...
   Pio has been successfully upgraded to 6.2.0!
   ```
   and called `update_core_packages()`.
9. `update_core_packages()` downloaded `https://github.com/pioarduino/scons/releases/download/4.11.1/scons-local-4.11.1.tar.gz` and wiped `~/.platformio/packages/tool-scons` (which was 4.8.1) with SCons 4.11.1.
10. SCons 4.11.1 refactored its tool modules and **dropped `SCons.Tool.FortranCommon`**.
11. When SCons linked the final firmware ELF, `SCons/Tool/linkCommon/__init__.py:132` executed `import SCons.Tool.FortranCommon`, which failed immediately with:
    `ModuleNotFoundError: No module named 'SCons.Tool.FortranCommon'`.

### Why Magnemite's Previous Pre-Check Failed
Magnemite added a check before the build to remove `tool-scons` if `FortranCommon.py` was missing. On a cold cache, `tool-scons` is absent before the build starts. The wipe does not happen before the build — it happens **mid-build** when `penv/bin/pio` launches stage 2!

---

## 3. Four-Layer Solution

### Layer 1: Pin Python to `"3.12"`
Pinned `python-version: "3.12"` across all workflows:
- `.github/workflows/nexus_build_firmware.yml`
- `.github/workflows/nexus_main_matrix.yml`
- `.github/workflows/nexus_draft_release.yml`
- `.github/workflows/nexus_build_one_target.yml`

Python 3.12 is pre-cached on all GitHub Actions runners, supported by ESP-IDF and pioarduino, and completely avoids 3.15.

### Layer 2: Pre-Create `~/.platformio/penv` Pinned to `6.1.19`
Before any `pio run` executes, CI initializes:
```bash
python -m venv "$HOME/.platformio/penv"
"$HOME/.platformio/penv/bin/pip" install --upgrade "uv>=0.1.0"
"$HOME/.platformio/penv/bin/pip" install "pioarduino==6.1.19" "platformio==6.1.19"
```
When `penv_setup.py` checks `installed_packages["pioarduino"]`, it finds version `6.1.19`. Because `SimpleSpec(">=6.1.19").match(semver("6.1.19"))` is `True`, `pioarduino` is **not** queued for installation. `uv pip install --upgrade` is never executed for `pioarduino`, so version 6.2.0 is never installed.

### Layer 3: Seed `~/.platformio/appstate.json`
```bash
echo '{"last_version": "6.1.19", "last_check": {"platformio_upgrade": 2147483647, "prune_system": 2147483647}}' > "$HOME/.platformio/appstate.json"
```
When `maintenance.py::after_upgrade()` checks `last_version`, `last_version == __version__` (`6.1.19 == 6.1.19`), so it returns `None` immediately. `update_core_packages()` is never triggered.

### Layer 4: `.piokeep` & Fallback `FortranCommon.py` Shim
- Touches `~/.platformio/packages/tool-scons/.piokeep` so PlatformIO's package cleaner ignores it.
- Generates a fallback `FortranCommon.py` shim in any SCons `Tool` directory:
  ```python
  """Fallback shim for SCons FortranCommon to prevent link crashes"""
  def isfortran(env, source): return False
  def add_all_to_env(env): pass
  def add_fortran_to_env(env): pass
  ```
- Cache keys bumped from `pio6119-` to `pio6119-py312-` to prevent restoring any previously poisoned runner caches.
- Integrated into both GitHub Actions workflows and `bin/build-esp32.sh`.
