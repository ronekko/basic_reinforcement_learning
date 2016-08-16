#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 21 15:52:18 2015

@author: sakurai
"""

import os
import shutil
import platform
from distutils.spawn import find_executable
import argparse
import subprocess

if __name__ == '__main__':
    #
    system = platform.system()
    assert system in ("Windows", "Linux"), "The OS must be Windows or Linux"
    if system == "Windows":
        exec_file = "vrep.exe"
        lib_file = "remoteApi.dll"
    elif system == "Linux":
        exec_file = "vrep.sh"
        lib_file = "remoteApi.so"

    # get the full path of the vrep executable file
    exec_path = find_executable(exec_file)
    assert exec_path is not None, "*{}* is not found.".format(exec_file)

    # parse command line option
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--visible", action="store_true",
                        help="launch v-rep as normal-mode, "
                             "instead of headless-mode")
    parser.add_argument("-d", "--debug", action="store_true",
                        help="launch v-rep as debug-mode")
    parser.add_argument("-p", "--port", default=19998,
                        help="port (default: 19998)")
    args = parser.parse_args()

    # make a command to launch v-rep with options specified
    h_opt = "-h " if not args.visible else ""
    g_opt = "REMOTEAPISERVERSERVICE_{}_{}_TRUE".format(
        args.port, args.debug).upper()
    scene_file = "vrep_basic_reinforcement_learning.ttt"
    scene_path = os.path.abspath(scene_file)

    vrep_command = "{} {} -g{} {}".format(exec_file, h_opt, g_opt, scene_path)
    print "command:", vrep_command

    # launch v-rep
    if system == "Windows":
        process = subprocess.Popen(vrep_command,
                                   creationflags=subprocess.CREATE_NEW_CONSOLE)
    elif system == "Linux":
        process = subprocess.Popen(vrep_command, shell=True)

    # prepare the required library files for remote API
    exec_dir = os.path.dirname(exec_path)
    api_dir = os.path.join(exec_dir, "programming", "remoteApiBindings")
    file_paths = [os.path.join(api_dir, "lib", "lib", "64Bit", lib_file),
                  os.path.join(api_dir, "python", "python", "vrep.py"),
                  os.path.join(api_dir, "python", "python", "vrepConst.py")]
    assert all(os.path.isfile(path) for path in file_paths), (
        "Library files for v-rep Remote API not found")
    for path in file_paths:
        shutil.copy(path, ".")

    print "exec_dir:", exec_dir
    print "api_dir:", api_dir
    print "library files:", "\n".join(file_paths)
