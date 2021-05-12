#!/usr/bin/python
# -*- coding: utf-8 -*-
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
from __future__ import with_statement  # Not required in Python 2.6 any more
import tokenize
import argparse
import numpy
import sys
import regex as re
import numpy as np
from pyquaternion import Quaternion as quat

import evaluate_ate_are

# import evaluators.associate

def get_user_system_time_memory_kb(filename_memory):
    f = open(filename_memory, 'r')
    text = f.read()

    text_regular_real_time = 'Elapsed (.*?)\n'
    found_real_time = re.findall(text_regular_real_time, text)
    splitted_real_time = found_real_time[0].split()
    splitted_real_time_hms = splitted_real_time[len(splitted_real_time) - 1]
    print('found real time: ', splitted_real_time_hms)
    secs = sum(float(x) * 60 ** i for i, x in enumerate(reversed(splitted_real_time_hms.split(':'))))

    print('seconds ', secs)
    text_regular_user_time = 'User time (.*?)\n'
    found_user_time = re.findall(text_regular_user_time, text)

    text_regular_system_time = 'System time (.*?)\n'
    found_system_time = re.findall(text_regular_system_time, text)

    text_regular_memory = 'Maximum resident set size (.*?)\n'
    found_memory = re.findall(text_regular_memory, text)

    splited_user_time = found_user_time[0].split()
    splited_system_time = found_system_time[0].split()
    splited_memory = found_memory[0].split()

    return splited_user_time[1], splited_system_time[1], splited_memory[1], secs


if __name__ == "__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory.
    ''')
    parser.add_argument('directory_root', help='directory with benchmark results')
    parser.add_argument('number_iterations', help='number of benchmark launches')
    parser.add_argument('groundtruth_file', help='full path to groundtruth poses')
    args = parser.parse_args()

    directory_root = args.directory_root
    groundtruth_file = args.groundtruth_file
    number_of_iterations = args.number_iterations

    ate_are_dict = []

    time_real_s = []
    time_s = []
    memory_mb = []

    for i in range(int(number_of_iterations)):
        print('\n\n\n======================================================================================')
        print('start comparing iteration number ', i)
        directory_iteration = directory_root + '/' + str(i)
        memory_file = directory_iteration + '/memory.txt'
        found_usr_time, found_system_time, found_memory_kb, real_time = get_user_system_time_memory_kb(memory_file)
        print(found_usr_time)
        print(type(found_usr_time))

        sum_time = float(found_usr_time) + float(found_system_time)
        sum_Mb = float(found_memory_kb) / 1024.0

        time_real_s.append(real_time)
        time_s.append(sum_time)
        memory_mb.append(sum_Mb)

        mean_ate, mean_are = evaluate_ate_are.evaluate(directory_iteration + '/ba.txt', groundtruth_file)
        print('iteration, ate, are: ', i, mean_ate, mean_are)
        ate_are_dict.append([mean_ate, i])
    sorted_ate_iterations = sorted(ate_are_dict)
    print(sorted_ate_iterations)

    iterations = int(number_of_iterations)
    iterations = min(iterations / 2, (iterations - 1) / 2)
    print('median iteration is ', sorted_ate_iterations[int(iterations)])


    print(time_real_s)

    print('time mean ', np.mean(time_s), ' s')
    print('time std ', np.std(time_s), ' s')

    print('REAL time mean ', np.mean(time_real_s), ' real s')
    print('REAL time std ', np.std(time_real_s), ' real s')

    print('memory mean ', np.mean(memory_mb), ' Mb')
    print('memory std ', np.std(memory_mb), ' Mb')