# extra_script.py
# Before each build, copy the project-local patched system_gd32f4xx.c over
# the SPL package copy so the 168 MHz / 8 MHz HXTAL clock configuration is used.

from shutil import copyfile
from os.path import join, isfile

Import("env")

framework_dir = env.PioPlatform().get_package_dir("framework-spl-gd32")
variant = env.BoardConfig().get("build.spl_series").lower()
spl_system = join(framework_dir, "gd32", "cmsis", "variants", variant, "system_gd32f4xx.c")
local_system = join(env["PROJECT_DIR"], "Src", "system_gd32f4xx.c")

assert isfile(local_system), "Project-local system_gd32f4xx.c not found"

def patch_system_file(*args, **kwargs):
    copyfile(local_system, spl_system)

env.AddPreAction("$BUILD_DIR/FrameworkCMSISVariant/libFrameworkCMSISVariant.a", patch_system_file)
