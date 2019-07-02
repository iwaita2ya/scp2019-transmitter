Import("env")
import lpc_checksum

# check following url for more detail
# http://docs.platformio.org/en/latest/projectconf/advanced_scripting.html#before-pre-and-after-post-actions

# Dump build environment (for debug)
# print env.Dump()

# ----------------------------------------------
# Custom actions when building program/firmware
# ----------------------------------------------
def before_buildprog(source, target, env):
    print "***** before_buildprog *****"
    # do some actions

    # call Node.JS or other script
    # env.Execute("node --version")

def after_buildprog(source, target, env):
    print "***** after_buildprog *****"
    print "SOURCES:", map(str, source) # ['.pioenvs/lpc11u35/firmware.bin']

    add_checksum(".pioenvs/lpc11u35/firmware.bin")

# add checksum
def add_checksum(source):
    print "source:" + source
    checksum = lpc_checksum.checksum(source)
    print "checksum:" + str(checksum)

# ------------------------------------------
# Custom actions for specific files/objects
# ------------------------------------------
#env.AddPreAction("$BUILD_DIR/${PROGNAME}.elf", [callback1, callback2,...])
#env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", callback...)

# ----------------------------------------------------------------------------
# custom action before building SPIFFS image. For example, compress HTML, etc.
# ----------------------------------------------------------------------------
#env.AddPreAction("$BUILD_DIR/spiffs.bin", callback...)
#env.AddPostAction("$BUILD_DIR/spiffs.bin", callback...)
#env.AddPostAction("$BUILD_DIR/firmware.bin", add_checksum)

# -------------------------------------
# custom action for project's main.cpp
# -------------------------------------
#env.AddPostAction("$BUILD_DIR/src/main.cpp.o", callback...)

env.AddPreAction("buildprog", before_buildprog)
env.AddPostAction("buildprog", after_buildprog)
