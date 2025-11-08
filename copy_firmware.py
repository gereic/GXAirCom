import os
Import("env", "projenv")
#from shutil import copyfile
import shutil

def get_build_flag_value(flag_name):
    build_flags = env.ParseFlags(env['BUILD_FLAGS'])
    flags_with_value_list = [build_flag for build_flag in build_flags.get('CPPDEFINES') if type(build_flag) == list]
    defines = {k: v for (k, v) in flags_with_value_list}
    return defines.get(flag_name)

def filecopy(source,dest):
    print("********  copy file " + source + " to " + dest + " *******")
    shutil.copy(source, dest)

def copy_file(*args, **kwargs):
    print("Copying file output to project directory...")
    version = get_build_flag_value("VERSION")
    version = version[1:-1]
    target = str(kwargs['target'][0])
    savename = target.split(os.path.sep)[-1]   # name of environment
    platform = target.split(os.path.sep)[-2]
    filename = target.split(os.path.sep)[-1]
    #print(target.split(os.path.sep)[-1])    
    #print(target.split(os.path.sep)[-2])    
    #print(target.split(os.path.sep)[-3])    
    #print(version)
    dirPath = 'bin/{}'.format(platform);
    if not os.path.exists(dirPath):
        os.mkdir(dirPath)
    dirPath = 'bin/{}/{}'.format(platform,version);
    if not os.path.exists(dirPath):
        os.mkdir(dirPath)
    source = target
    target = 'bin/{}/{}/{}'.format(platform,version,filename)
    filecopy(source,target)
    if filename == "firmware.bin":
        #copy bootloader for old compiler-version (3.4.0)
        source = '.pio/build/{}/bootloader.bin'.format(platform)
        target = 'bin/{}/{}/bootloader.bin'.format(platform,version)
        if not os.path.exists(source):
            source = 'bin/bootloader_dio_40m.bin'
        filecopy(source,target)
        
        #source = target
        #target = 'bin/firmware_{}_{}.bin'.format(version,platform)
        #filecopy(source,target)
        f = open("bin/_version.txt", "w")
        f.write(version)
        f.close()
        print("*************  Done. **********************")

env.AddPostAction("$BUILD_DIR/bootloader.bin", copy_file)
env.AddPostAction("$BUILD_DIR/partitions.bin", copy_file)
env.AddPostAction("$BUILD_DIR/firmware.bin", copy_file)
env.AddPostAction("$BUILD_DIR/spiffs.bin", copy_file)
