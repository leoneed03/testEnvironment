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

def evaluate(first_file, second_file):
    first_list = associate.read_file_list(first_file)
    second_list = associate.read_file_list(second_file)

    matches = associate.associate(first_list, second_list, 0.0, 0.02)

    if len(matches) < 2:
        sys.exit(
            "Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")

    first_xyz = numpy.matrix([[float(value) for value in first_list[a][0:3]] for a, b in matches]).transpose()
    second_xyz = numpy.matrix(
        [[float(value) * float(1.0) for value in second_list[b][0:3]] for a, b in matches]).transpose()

    first_quat = numpy.matrix([[float(value) for value in first_list[a][3:7]] for a, b in matches])
    second_quat = numpy.matrix(
        [[float(value) * float(1.0) for value in second_list[b][3:7]] for a, b in matches])

    first_quats = []

    rot, trans, trans_error = align(second_xyz, first_xyz)

    for quat_1 in first_quat:
        quat_1_array = numpy.squeeze(numpy.asarray(quat_1))

        quat_constr = quat(quat_1_array[3], quat_1_array[0], quat_1_array[1], quat_1_array[2])
        first_quats.append(quat_constr)

    counter = 0

    rot_errors = []

    for quat_1 in second_quat:
        quat_1_array = numpy.squeeze(numpy.asarray(quat_1))
        quat_constr = quat(quat_1_array[3], quat_1_array[0], quat_1_array[1], quat_1_array[2])
        orientation_aligned = first_quats[counter].inverse.rotation_matrix * rot * quat_constr.rotation_matrix
        rot_errors.append(angle(orientation_aligned))
        counter += 1

    first_stamps = sorted(first_list)
    second_stamps = sorted(second_list)
    second_xyz_full = numpy.matrix(
        [[float(value) for value in second_list[b][0:3]] for b in second_stamps]).transpose()

    print("compared_pose_pairs " + str(len(trans_error)) + " pairs")

    print("alignment transformation R + t is")
    print(rot)
    print(trans)

    print("absolute_translational_error.rmse " + str(numpy.sqrt(
        numpy.dot(trans_error, trans_error) / len(trans_error))) + " m")
    print("absolute_translational_error.mean " + str(numpy.mean(trans_error)) + " m")
    print("absolute_translational_error.median " + str(numpy.median(trans_error)) + " m")
    print("absolute_translational_error.std " + str(numpy.std(trans_error)) + " m")
    print("absolute_translational_error.min " + str(numpy.min(trans_error)) + " m")
    print("absolute_translational_error.max " + str(numpy.max(trans_error)) + " m")

    print()

    print("absolute_rotational_error.rmse " + str(numpy.sqrt(
        numpy.dot(rot_errors, rot_errors) / len(rot_errors))) + " rad")
    print("absolute_rotational_error.mean " + str(numpy.mean(rot_errors)) + " rad")
    print("absolute_rotational_error.median " + str(numpy.median(rot_errors)) + " rad")
    print("absolute_rotational_error.std " + str(numpy.std(rot_errors)) + " rad")
    print("absolute_rotational_error.min " + str(numpy.min(rot_errors)) + "  rad")
    print("absolute_rotational_error.max " + str(numpy.max(rot_errors)) + " rad")

    return numpy.mean(trans_error), numpy.mean(rot_errors)
