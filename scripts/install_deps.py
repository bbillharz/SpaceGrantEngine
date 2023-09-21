#!/usr/bin/env python3

import subprocess
import os
import sys

def install_apt_packages(packages):
    if not isinstance(packages, list):
        packages = [packages]
    subprocess.check_call(["sudo", "apt", "install", "-y"] + packages)


ros_distro = os.getenv("ROS_DISTRO")
if ros_distro is None:
    print("ROS_DISTRO is not set. Has ROS been sourced?")
    exit(1)

subprocess.check_call(["sudo", "apt", "update"])

# Install misic tools
install_apt_packages(["nano", "wget", "curl"])

# Install deps (system)
install_apt_packages(
    [
        f"ros-{ros_distro}-depthai",
        "python3-cv-bridge",
        "python3-venv"
    ]
)

if "--apt_only" in sys.argv:
    exit()

subprocess.check_call(["python3", "-m", "venv", "venv"])

def install_pip_packages(packages, check_return_code=True):
    if not isinstance(packages, list):
        packages = [packages]
    command = ["./venv/bin/pip3", "install"] + packages + ["--upgrade"]
    if check_return_code:
        subprocess.check_call(command)
    else:
        subprocess.run(command)

# Upgrade pip for compatibility with newer packages
install_pip_packages("pip")

install_pip_packages(
    [
        "black",
        "isort",
        "ruff",
        "pyserial",
        "numpy",
        "scipy",
        "depthai",
        "opencv-contrib-python==4.8.0.74",
        "oakutils",
    ]
)
install_pip_packages(["./extern/openVO", "./extern/steamcontroller"], check_return_code=False)

install_pip_packages("RPi.GPIO", check_return_code=False)
