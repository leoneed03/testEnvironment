#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Requirements:
# sudo apt-get install python-argparse

"""
This script computes the absolute trajectory error from the ground truth
trajectory and the estimated trajectory.
"""

import sys
# from random import random

import random
import numpy
from argparse import ArgumentParser
import argparse
# import associate
import subprocess
import re
import threading
import time
import os


def get_bash_output_and_error(bash_command_file, path_to_console_log_file):
    bash_command = 'sh ' + bash_command_file
    # exec(open(bash_command_file).read())
    with open(path_to_console_log_file, 'w') as f:
        subprocess.call(bash_command.split())


def benchmark(dataset_root, out_poses_dir, intrinsics_depth_divider_string,
              out_poses_files='irls.txt ba.txt gt.txt',
              iterations=6):
    print("START")
    process_name_example = '/usr/bin/time --verbose /home/leoneed/CLionProjects/GDR/cmake-build-debug/reconstructorTUM'  # args_parsed.process_name

    usr_bin_time_log_file = 'memory.txt'
    console_log = 'console_log.txt'

    for i in range(iterations):
        out_poses_dir_i = out_poses_dir + '/' + str(i)
        os.makedirs(out_poses_dir_i)
        process_name_and_args_example = process_name_example + ' ' + dataset_root + ' ' + out_poses_dir_i + ' ' + intrinsics_depth_divider_string + ' ' + out_poses_files
        command_to_execute = '(' + process_name_and_args_example + ') 2>> ' + out_poses_dir_i + '/' + usr_bin_time_log_file
        full_console_log = out_poses_dir_i + '/' + console_log

        print(command_to_execute)
        command_file_name = 'command.sh'
        command_file = open(command_file_name, 'w')
        command_file.write(command_to_execute)

        get_bash_output_and_error(command_file_name, full_console_log)

    print('DONE ')


if __name__ == "__main__":

    # dataset_root_arg_desk = '/home/leoneed/Desktop/datasets/freiburg/rgbd_dataset_freiburg1_desk'
    # out_poses_dir_arg_desk = '/home/leoneed/Desktop/results/GDR/desk1_full'
    # intrinsics_depth_divider_string_arg_desk = '517.3 516.5 318.6 255.3 5000.0'
    # benchmark(dataset_root_arg_desk, out_poses_dir_arg_desk, intrinsics_depth_divider_string_arg_desk)
    #
    # dataset_root_arg_xyz = '/home/leoneed/Desktop/datasets/freiburg/fr2_xyz_sampled_456_8'
    # out_poses_dir_arg_xyz = '/home/leoneed/Desktop/results/GDR/xyz2_sampled_456_8'
    # intrinsics_depth_divider_string_arg_xyz = '520.9 521.0 325.1 249.7 5000.0'
    # benchmark(dataset_root_arg_xyz, out_poses_dir_arg_xyz, intrinsics_depth_divider_string_arg_xyz)
    #
    # dataset_root_arg_office = '/home/leoneed/Desktop/datasets/freiburg/fr3_office_sampled_498_5'
    # out_poses_dir_arg_office = '/home/leoneed/Desktop/results/GDR/office3_new'
    # intrinsics_depth_divider_string_arg_office = '535.4 539.2 320.1 247.6 5000.0'
    # benchmark(dataset_root_arg_office, out_poses_dir_arg_office, intrinsics_depth_divider_string_arg_office)

    # dataset_root_arg_office = '/home/leoneed/Desktop/datasets/freiburg/fr3_office_sampled_829_3'
    # out_poses_dir_arg_office = '/home/leoneed/Desktop/results/GDR/office3_sampled_829_3'
    # intrinsics_depth_divider_string_arg_office = '535.4 539.2 320.1 247.6 5000.0'
    # benchmark(dataset_root_arg_office, out_poses_dir_arg_office, intrinsics_depth_divider_string_arg_office)

    dataset_root_arg_office = '/home/leoneed/Desktop/datasets/freiburg/rgbd_dataset_freiburg3_long_office_household'
    out_poses_dir_arg_office = '/home/leoneed/Desktop/results/GDR/office3_full'
    intrinsics_depth_divider_string_arg_office = '535.4 539.2 320.1 247.6 5000.0'
    benchmark(dataset_root_arg_office, out_poses_dir_arg_office, intrinsics_depth_divider_string_arg_office)
