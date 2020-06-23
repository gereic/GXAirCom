Import("env", "projenv")
from shutil import copyfile

def copy_file(*args, **kwargs):
    print("Copying file output to project directory...")
    target = str(kwargs['target'][0])
    savename = target.split('\\')[-1]   # name of environment
    savefile = 'bin/{}'.format(savename)
    #print(savename)
    print(savefile)
    copyfile(target, savefile)
    print("Done.")

env.AddPostAction("$BUILD_DIR/partitions.bin", copy_file)
env.AddPostAction("$BUILD_DIR/firmware.bin", copy_file)
env.AddPostAction("$BUILD_DIR/spiffs.bin", copy_file)