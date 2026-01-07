# tools/enable_micro_ros.py
# Dynamically locate and run micro_ros_arduino's platformio-build.py
# Works even if library is not yet downloaded on the first run.

import os
from SCons.Script import AlwaysBuild

def _try_import_microros_script(env):
    # 1) vendored path (lib/micro_ros_arduino/...)
    vendored = os.path.join(env['PROJECT_DIR'], 'lib', 'micro_ros_arduino', 'scripts', 'platformio-build.py')
    # 2) libdeps path (.pio/libdeps/<env_name>/micro_ros_arduino/...)
    libdeps = os.path.join(env['PROJECT_LIBDEPS_DIR'], env['PIOENV'], 'micro_ros_arduino', 'scripts', 'platformio-build.py')

    for cand in (vendored, libdeps):
        if os.path.isfile(cand):
            print("[enable_micro_ros] Using micro_ros script:", cand)
            # Run the upstream script with our build env
            env.SConscript(cand, exports="env")
            return True

    print("[enable_micro_ros] micro_ros_arduino script not found yet; will try again before link.")
    return False

def _prelink_hook(source, target, env):
    # Try again right before linking (library should be downloaded by now)
    _try_import_microros_script(env)

def generate(env):
    # Try immediately
    found = _try_import_microros_script(env)
    if not found:
        # Try again before linking the firmware.elf
        # (This hook runs late enough that lib_deps are already present.)
        env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf", _prelink_hook)

def exists(env):
    return True
