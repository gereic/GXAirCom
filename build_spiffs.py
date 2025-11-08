Import("env")
from SCons.Script import DefaultEnvironment
import os


def before_all_build(source, target, env):
    print("\n=== [PRE-BUILD] Erstelle SPIFFS Image ===")
    #env.Execute("pio run --target buildfs")
    #env.Execute(env.Alias("buildfs"))
    env.Execute(
        f"pio run -e {env['PIOENV']} --target buildfs"
    )    

env.AddPreAction("checkprogsize", before_all_build)
