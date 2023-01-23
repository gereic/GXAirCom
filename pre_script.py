import os
from shutil import copyfile

igc_key_path = "igc_check/key.pk"
igc_h_path = "igc_check/igc_key.h"

print("Pre Action script ...")

# if key does not exists create a fake one
if not os.path.exists(igc_key_path):
  print("Missing IGC PK, creating fake one.")
  f = open(igc_key_path,'w')
  f.write('#define SHAPRIVATEKEY "myawesomepkey"')
  f.close()

print("********  copy file " + igc_key_path + " to " + igc_h_path + " *******")
copyfile(igc_key_path, igc_h_path)
print("Done.")

