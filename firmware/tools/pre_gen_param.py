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

param_yaml = os.path.join(MADFLIGHT_DIR, "src", "cfg", "param.yaml")
if not os.path.isfile(param_yaml):
    print("WARNING: %s not found, skipping parameter generation" % param_yaml)
else:
    gen_param.generate(param_yaml)
