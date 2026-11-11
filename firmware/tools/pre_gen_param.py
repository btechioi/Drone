"""PlatformIO pre-build script.

Regenerates madflight's ``src/cfg/param.yaml.h`` from ``src/cfg/param.yaml`` so
that the generated parameter enums always match the yaml source of truth.

The actual generator lives in ``tools/gen_param.py`` and can also be run by
hand::

    python3 tools/gen_param.py
"""

import os
import sys

Import("env")  # noqa: F821  (injected by PlatformIO/SCons)

PROJECT_DIR = env["PROJECT_DIR"]
TOOLS_DIR = os.path.join(PROJECT_DIR, "tools")
MADFLIGHT_DIR = os.path.normpath(os.path.join(PROJECT_DIR, "..", "madflight"))

sys.path.insert(0, TOOLS_DIR)

import gen_param  # noqa: E402

# Refuse to build the pinned receiver with its known broken CRC check. Keep
# the submodule patch explicit rather than silently modifying a checkout.
serial_header = os.path.join(MADFLIGHT_DIR, "src", "rcl", "RclGizmoSerial.h")
if not os.path.isfile(serial_header):
    raise RuntimeError("madflight submodule is missing; run git submodule update --init --recursive")
with open(serial_header, encoding="utf-8") as stream:
    receiver = stream.read()
if ("frame[0] = cmd;\n          uint16_t crc_calc" not in receiver or
        "if(!_handle_frame())" not in receiver or
        "!std::isfinite(throttle)" not in receiver):
    raise RuntimeError(
        "Unsafe or unrecognized madflight SERIAL receiver: apply "
        "firmware/patches/madflight-serial-crc.patch inside the madflight submodule "
        "and review receiver safety before use."
    )

param_yaml = os.path.join(MADFLIGHT_DIR, "src", "cfg", "param.yaml")
if not os.path.isfile(param_yaml):
    raise RuntimeError("madflight submodule is missing; run git submodule update --init --recursive")
else:
    gen_param.generate(param_yaml)
