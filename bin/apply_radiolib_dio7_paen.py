#!/usr/bin/env python3
"""Apply the NiceRF LR2021 DIO7 PA-enable patch to the env's RadioLib source.

The vendor LoRa2021F33-2G4 demo drives DIO7 as a static high GPIO (2.4 GHz PA
VCC enable). RadioLib's setRfSwitchTable() leaves DIO7 as an RF-switch output,
so the variant build passes -DLR2021_DIO7_PAEN_ALWAYS_HIGH and this script
inserts the matching block after the RF-switch table is programmed.

Run as a PlatformIO `pre:` extra script. If the library is not installed yet
(a clean build), it warns and leaves the build to continue unpatched; run the
env again (or `pio pkg install`) and the patch will land.
"""

Import("env")

import os
import sys

MARKER = "LR2021_DIO7_PAEN_ALWAYS_HIGH"

OLD = """    (void)this->setDioRfSwitchConfig(dioNum + 5, dioConfigs[dioNum]);
  }
}
"""

NEW = """    (void)this->setDioRfSwitchConfig(dioNum + 5, dioConfigs[dioNum]);
  }

#ifdef LR2021_DIO7_PAEN_ALWAYS_HIGH
  // NiceRF LoRa2021F33-2G4: DIO7 is the 2.4 GHz PA VCC enable. The vendor demo
  // drives it as a static high GPIO with pull-up, not as an RF-switch output.
  (void)this->setDioFunction(7, RADIOLIB_LR2021_DIO_FUNCTION_GPIO_OUTPUT_HIGH, RADIOLIB_LR2021_DIO_SLEEP_PULL_UP);
#endif
}
"""

libdeps = env.subst("$PROJECT_LIBDEPS_DIR")
pioenv = env.subst("$PIOENV")
path = os.path.join(libdeps, pioenv, "RadioLib", "src", "modules", "LR2021", "LR2021_config.cpp")

if not os.path.isfile(path):
    sys.stderr.write(f"RadioLib DIO7 PAEN patch: {path} not installed yet; skipping\n")
else:
    with open(path, "r", encoding="utf-8") as f:
        content = f.read()
    if MARKER in content:
        print(f"RadioLib DIO7 PAEN patch: already applied ({pioenv})")
    elif OLD not in content:
        sys.stderr.write(f"RadioLib DIO7 PAEN patch: anchor not found in {path}; skipping\n")
    else:
        with open(path, "w", encoding="utf-8") as f:
            f.write(content.replace(OLD, NEW, 1))
        print(f"RadioLib DIO7 PAEN patch: applied ({pioenv})")
