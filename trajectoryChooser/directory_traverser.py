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
import argparse
import numpy
import sys
from pyquaternion import Quaternion as quat

import evaluate_ate_are
# import evaluators.associate



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

    for i in range(int(number_of_iterations)):
        print('\n\n\n======================================================================================')
        print('start comparing iteration number ', i)
        mean_ate, mean_are = evaluate_ate_are.evaluate(directory_root + '/' + str(i) + '/ba.txt', groundtruth_file)
        print('iteration, ate, are: ', i, mean_ate, mean_are)
        ate_are_dict.append([mean_ate, i])
    sorted_ate_iterations = sorted(ate_are_dict)
    print(sorted_ate_iterations)

    iterations = int(number_of_iterations)
    iterations = min(iterations / 2, (iterations - 1) / 2)
    print('median iteration is ', sorted_ate_iterations[int(iterations)])