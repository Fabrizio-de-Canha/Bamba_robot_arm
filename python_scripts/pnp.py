#!/usr/bin/env python

import rospy, sys
import numpy as np
from scipy.optimize import fmin_slsqp


def position(joint_arr, pos_desired_arr):
    azimuth = np.arctan(pos_desired_arr[1]/pos_desired_arr[0])

    # length_0 = 0.082
    # length_1 = 0.068
    # length_2 = 0.0525
    # length_3 = 0.0525
    # length_4 = 0

    length_0 = 0.082
    length_1 = 0.092
    length_2 = 0.057
    length_3 = 0
    length_4 = 0
    ghost_length = 0.1*np.sin(0.52) + 0.025

    global_1 = joint_arr[0]
    global_2 = global_1 + joint_arr[1]
    global_3 = global_2 + joint_arr[2]
    global_4 = global_3 + joint_arr[3]

    link_1_x = -1*length_1 * np.sin(global_1) * np.cos(azimuth)
    link_2_x = -1*length_2 * np.sin(global_2) * np.cos(azimuth)
    link_3_x = -1*length_3 * np.sin(global_3) * np.cos(azimuth)
    link_4_x = -1*length_4 * np.sin(global_4) * np.cos(azimuth)

    if np.sin(global_1) >= 0:
        sign = 1
    else:
        sign = -1

    link_1_z = length_1 * np.cos(global_1)
    link_2_z = length_2 * np.cos(global_2)
    link_3_z = length_3 * np.cos(global_3)
    link_4_z = length_4 * np.cos(global_4)

    x_pos = link_1_x + link_2_x + link_3_x + link_4_x + length_2*-1*np.cos(azimuth)*sign - 1*np.cos(azimuth)*sign*ghost_length
    z_pos = link_1_z + link_2_z + link_3_z + link_4_z + length_0

    return [x_pos, 0, z_pos]

def position_correction(joint_arr, pos_desired_arr):
    azimuth = np.arctan(pos_desired_arr[1]/pos_desired_arr[0])

    # length_0 = 0.082
    # length_1 = 0.068
    # length_2 = 0.0525
    # length_3 = 0.0525
    # length_4 = 0

    length_0 = 0.082
    length_1 = 0.092
    length_2 = 0.057
    length_3 = 0
    length_4 = 0
    error = -0.015
    ghost_length = 0.1*np.sin(0.52) + 0.025

    global_1 = joint_arr[0]
    global_2 = global_1 + joint_arr[1]
    global_3 = global_2 + joint_arr[2]
    global_4 = global_3 + joint_arr[3]

    link_1_x = -1*length_1 * np.sin(global_1) * np.cos(azimuth)
    link_2_x = -1*length_2 * np.sin(global_2) * np.cos(azimuth)
    link_3_x = -1*length_3 * np.sin(global_3) * np.cos(azimuth)
    link_4_x = -1*length_4 * np.sin(global_4) * np.cos(azimuth)

    if np.sin(global_1) >= 0:
        sign = 1
    else:
        sign = -1

    link_1_z = length_1 * np.cos(global_1)
    link_2_z = length_2 * np.cos(global_2)
    link_3_z = length_3 * np.cos(global_3)
    link_4_z = length_4 * np.cos(global_4)

    x_pos = link_1_x + link_2_x + link_3_x + link_4_x + length_2*-1*np.cos(azimuth)*sign - 1*error*np.cos(azimuth)*sign - 1*sign*np.cos(azimuth)*ghost_length
    z_pos = link_1_z + link_2_z + link_3_z + link_4_z + length_0

    return [x_pos, 0, z_pos]


def error_vector(joint_arr, pos_desired_arr):
    vector = pos_desired_arr - np.array(position(joint_arr, pos_desired_arr))
    return (vector[0] ** 2 + vector[2] ** 2) ** 0.5 * 1000

def error_vector_correction(joint_arr, pos_desired_arr):
    vector = pos_desired_arr - np.array(position_correction(joint_arr, pos_desired_arr))
    return (vector[0] ** 2 + vector[2] ** 2) ** 0.5 * 1000

def set_position_target(joint_arr, endeff):
    if not joint_arr[2] < 1000:
        print('========== ABORT: NO VALID SOLUTION FOUND ==========')
        return 1
    list1 = list(joint_arr)
    list1.append(endeff)
    target = np.array(list1)
    np.savetxt('Desktop/cv_files/target.txt', target)
    return None

def go():
    rospy.sleep(2)
    return None

arr_1 = np.array([0, 0, 0])
np.savetxt('Desktop/cv_files/coord.txt', arr_1), np.savetxt('Desktop/cv_files/place.txt', arr_1)

main_group = 'arm'
endeffector = 'endeff'

endeffector_frame = 'link_4'

endeffector_open = 2
endeffector_closed = 0.35

endeffector_joints = ['joint_5', 'joint_slave']

reference_frame = 'base'
np.savetxt('Desktop/cv_files/executed.txt', np.array([0]))

def joint_state_generator():
        y = 'y'
        n = 'n'
        zero_arr = np.array([0, 0, -0.2, 0.1, 0])

        while True:
            print('Proceeding to zero position')
            joints = zero_arr
            endeff = endeffector_closed
            set_position_target(joints, endeff)
            go()
            print('Zero position reached')

            print('Waiting for co-ordinates')
            pos_desired_arr = np.loadtxt('Desktop/cv_files/coord.txt')
            tvec_original = np.loadtxt('Desktop/cv_files/tvec_original.txt')
            while pos_desired_arr.any() == 0:
                pos_desired_arr = np.loadtxt('Desktop/cv_files/coord.txt')
                tvec_original = np.loadtxt('Desktop/cv_files/tvec_original.txt')
                rospy.sleep(1)
            place = np.loadtxt('Desktop/cv_files/place.txt')

            print('Planning for pick position')
            joint_arr = np.array([0, 0, 0, 0])
            a = (-1.37, 1.77)
            b = (-1.55, 1.55)
            temp_goal = fmin_slsqp(error_vector, joint_arr, eqcons = [error_vector], args = (pos_desired_arr,), iter = 10000, bounds = [a, a, a, b], acc = 1e-12)
            joint_0 = np.arctan(pos_desired_arr[1]/pos_desired_arr[0])
            joint_goal = [joint_0]
            for i in temp_goal:
                joint_goal.append(i)
            sum = 0
            for i in range(2):
                sum += joint_goal[i + 1]
            while sum > 2*np.pi:
                sum += -2*np.pi
            joint_3 = -1*sum + np.pi/2

            if joint_3 < -1.57:
                joint_3 += 3.14

            if joint_3 > 1.57:
                joint_3 += -3.14
            joint_goal[3] = joint_3

            if pos_desired_arr[0] < 0:
                joint_goal[4] = 1.15
            else:
                joint_goal[4] = -1.47

            if place == 2:
                joint_goal = -1*np.array(joint_goal)

            joint_goal += zero_arr

            print('joint_goal = {}'.format(joint_goal))
            
            joints = joint_goal
            endeff = endeffector_open
            mode = set_position_target(joints, endeff)
            if mode == 1:
                var = input('Planning failed for this position. Input any number to continue')
                continue
            go()
            print('Preliminary position reached')
            print('Correcting position')
            rospy.sleep(1.5)

            tvec_corr = np.loadtxt('Desktop/cv_files/tvec_corr.txt')
            print('original = {}, actual = {}'.format(tvec_original, tvec_corr))
            correction = (tvec_corr - tvec_original)
            correction[0] = -1*correction[0]
            correction[2] = 0.02
            if place == 2:
                correction = -1*correction
                correction[2] = 0.02
            print('pos_desired_arr = {}'.format(pos_desired_arr))
            print('correction = {}'.format(correction))
            pos_desired_arr = pos_desired_arr - correction

            temp_goal = fmin_slsqp(error_vector_correction, joint_arr, eqcons = [error_vector_correction], args = (pos_desired_arr,), iter = 10000, bounds = [a, a, a, b], acc = 1e-12)
            joint_0 = np.arctan(pos_desired_arr[1]/pos_desired_arr[0])
            joint_goal = [joint_0]
            for i in temp_goal:
                joint_goal.append(i)
            sum = 0

            for i in range(2):
                sum += joint_goal[i + 1]
            while sum > 2*np.pi:
                sum += -2*np.pi
            joint_3 = -1*sum + np.pi/2

            if joint_3 < -1.57:
                joint_3 += 3.14

            if joint_3 > 1.57:
                joint_3 += -3.14
            joint_goal[3] = joint_3

            if pos_desired_arr[0] < 0:
                joint_goal[4] = 1.15
            else:
                joint_goal[4] = -1.37

            if place == 2:
                joint_goal = -1 * np.array(joint_goal)

            joint_goal += zero_arr

            joints = joint_goal
            endeff = endeffector_open
            set_position_target(joints, endeff)
            go()

            joints = joint_goal
            endeff = endeffector_closed
            set_position_target(joints, endeff)
            go()

            joints = zero_arr
            endeff = endeffector_closed
            set_position_target(joints, endeff)
            go()

            print('Waiting for coordinates')
            joint_arr = np.array([0, 0, 0, 0])
            arr_1 = np.array([0, 0, 0])
            np.savetxt('Desktop/cv_files/coord_place.txt', arr_1)
            pos_place_arr = np.loadtxt('Desktop/cv_files/coord_place.txt')
            while pos_place_arr.any() == 0:
                pos_place_arr = np.loadtxt('Desktop/cv_files/coord_place.txt')
                tvec_original = np.loadtxt('Desktop/cv_files/tvec_original_place.txt')
                rospy.sleep(1)
            print('Position aqcuired')
            place = np.loadtxt('Desktop/cv_files/place_place.txt')
            a = (-1.57, 1.57)
            pos_desired_arr = pos_place_arr
            temp_goal = fmin_slsqp(error_vector, joint_arr, eqcons=[error_vector], args=(pos_desired_arr,), iter=10000,
                                   bounds=[a, a, a, b], acc = 1e-12)
            joint_0 = np.arctan(pos_desired_arr[1]/pos_desired_arr[0])
            joint_goal = [joint_0]
            print('temp_goal = ', temp_goal)
            for i in temp_goal:
                joint_goal.append(i)
            sum = 0

            for i in range(2):
                sum += joint_goal[i + 1]
            while sum > 2*np.pi:
                sum += -2*np.pi
            joint_3 = -1*sum + np.pi/2

            if joint_3 < -1.57:
                joint_3 += 3.14

            if joint_3 > 1.57:
                joint_3 += -3.14
            joint_goal[3] = joint_3

            if pos_desired_arr[0] < 0:
                joint_goal[4] = 1.15
            else:
                joint_goal[4] = -1.37

            if place == 2:
                joint_goal = -1 * np.array(joint_goal)

            joint_goal += zero_arr

            joints = joint_goal
            endeff = endeffector_closed
            set_position_target(joints, endeff)
            go()
            rospy.sleep(1.5)

            print('Preliminary target reached: Correcting position')

            tvec_corr = np.loadtxt('Desktop/cv_files/tvec_corr.txt')
            print('original = {}, actual = {}'.format(tvec_original, tvec_corr))
            correction = (tvec_corr - tvec_original)
            correction[0] = -1 * correction[0]
            correction[2] = 0.02
            if place == 2:
                correction = -1 * correction
            print('pos_desired_arr = {}'.format(pos_desired_arr))
            print('correction = {}'.format(correction))
            pos_desired_arr = pos_desired_arr - correction

            temp_goal = fmin_slsqp(error_vector_correction, joint_arr, eqcons=[error_vector_correction],
                                   args=(pos_desired_arr,), iter=10000, bounds=[a, a, a, b], acc=1e-12)
            joint_0 = np.arctan(pos_desired_arr[1] / pos_desired_arr[0])
            joint_goal = [joint_0]
            for i in temp_goal:
                joint_goal.append(i)
            sum = 0

            for i in range(2):
                sum += joint_goal[i + 1]
            while sum > 2 * np.pi:
                sum += -2 * np.pi
            joint_3 = -1 * sum + np.pi / 2

            if joint_3 < -1.57:
                joint_3 += 3.14

            if joint_3 > 1.57:
                joint_3 += -3.14
            joint_goal[3] = joint_3

            if pos_desired_arr[0] < 0:
                joint_goal[4] = 1.15
            else:
                joint_goal[4] = -1.37

            if place == 2:
                joint_goal = -1 * np.array(joint_goal)

            joint_goal += zero_arr

            joints = joint_goal
            endeff = endeffector_closed
            set_position_target(joints, endeff)
            go()

            joints = joint_goal
            endeff = endeffector_open
            set_position_target(joints, endeff)
            go()

            print('Proceeding to zero position')

            joints = zero_arr
            endeff = endeffector_closed
            set_position_target(joints, endeff)
            go()

            print('Zero position reached')

            putin = input('Run again [y/n]?')
            if putin == 'y':
                print('Running again')
            elif putin == 'n':
                print('Stopping')
                break
            else:
                print('Invalid input: Stopping')
                break

joint_state_generator()

