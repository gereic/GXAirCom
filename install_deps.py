import subprocess
import os

def install_pip_packages():
    print("Checking Python packages...")
    subprocess.call(['pip', 'install', '-r', 'requirements.txt'])

def install_npm_packages():
    bundle_path = os.path.join(os.getcwd(), 'bundle')
    if os.path.exists(os.path.join(bundle_path, 'package.json')):
        print("Installing Node.js packages in /bundle...")
        subprocess.call(['npm', 'install'], cwd=bundle_path, shell=True)    

#install_pip_packages()
install_npm_packages() #nodejs has to installed on windows system