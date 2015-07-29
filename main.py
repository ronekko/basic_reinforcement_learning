# -*- coding: utf-8 -*-
"""
Created on Sat Jul 18 15:44:19 2015

@author: ryuhei
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import vrep
import contexttimer

if __name__ == '__main__':
    try:
        client_id
    except NameError:
        client_id = -1
    e = vrep.simxStopSimulation(client_id, vrep.simx_opmode_oneshot_wait)
    vrep.simxFinish(-1)
    client_id = vrep.simxStart('127.0.0.1', 19998, True, True, 5000, 5)

    assert client_id != -1, 'Failed connecting to remote API server'

    e = vrep.simxStartSimulation(client_id, vrep.simx_opmode_oneshot_wait)

    # print ping time
    sec, msec = vrep.simxGetPingTime(client_id)
    print "Ping time: %f" % (sec + msec / 1000.0)

    # Handles of body and wheels
    e, body = vrep.simxGetObjectHandle(client_id, 'body',
                                       vrep.simx_opmode_oneshot_wait)
    e, joint_0 = vrep.simxGetObjectHandle(client_id, 'joint_0',
                                          vrep.simx_opmode_oneshot_wait)
    e, joint_1 = vrep.simxGetObjectHandle(client_id, 'joint_1',
                                          vrep.simx_opmode_oneshot_wait)

    # Set the joint angles to the default position (0, 0)
    e = vrep.simxSetJointTargetPosition(client_id, joint_0, 0,
                                        vrep.simx_opmode_streaming)
    e = vrep.simxSetJointTargetPosition(client_id, joint_1, 0,
                                        vrep.simx_opmode_streaming)

    # this line is necessary to enable position recording
    e, body_pos = vrep.simxGetObjectPosition(client_id, body, -1,
                                             vrep.simx_opmode_streaming)
    # Wait until the robot is settled to the default position
    time.sleep(0.3)

    # enable synchronous mode
    vrep.simxSynchronous(client_id, True)

    position_history = []
    e, body_pos = vrep.simxGetObjectPosition(client_id, body, -1,
                                             vrep.simx_opmode_buffer)
    position_history.append(body_pos)

    joint_pos_history = []
    e, joint_0_pos = vrep.simxGetJointPosition(client_id, joint_0,
                                               vrep.simx_opmode_streaming)
    e, joint_1_pos = vrep.simxGetJointPosition(client_id, joint_1,
                                               vrep.simx_opmode_streaming)
    joint_pos_history.append([joint_0_pos, joint_1_pos])

    with contexttimer.Timer() as timer:
        for i in range(4):
            e = vrep.simxSetJointTargetPosition(client_id, joint_0, 0.5,
                                                vrep.simx_opmode_streaming)
            for t in range(3):
                vrep.simxSynchronousTrigger(client_id)
                e, body_pos = vrep.simxGetObjectPosition(
                    client_id, body, -1, vrep.simx_opmode_buffer)
                position_history.append(body_pos)
                e, joint_0_pos = vrep.simxGetJointPosition(
                    client_id, joint_0, vrep.simx_opmode_buffer)
                e, joint_1_pos = vrep.simxGetJointPosition(
                    client_id, joint_1, vrep.simx_opmode_buffer)
                joint_pos_history.append([joint_0_pos, joint_1_pos])

            e = vrep.simxSetJointTargetPosition(client_id, joint_1, 2.5,
                                                vrep.simx_opmode_streaming)
            for t in range(10):
                vrep.simxSynchronousTrigger(client_id)
                e, body_pos = vrep.simxGetObjectPosition(
                    client_id, body, -1, vrep.simx_opmode_buffer)
                position_history.append(body_pos)
                e, joint_0_pos = vrep.simxGetJointPosition(
                    client_id, joint_0, vrep.simx_opmode_buffer)
                e, joint_1_pos = vrep.simxGetJointPosition(
                    client_id, joint_1, vrep.simx_opmode_buffer)
                joint_pos_history.append([joint_0_pos, joint_1_pos])

            e = vrep.simxSetJointTargetPosition(client_id, joint_0, 0.0,
                                                vrep.simx_opmode_streaming)
            for t in range(2):
                vrep.simxSynchronousTrigger(client_id)
                e, body_pos = vrep.simxGetObjectPosition(
                    client_id, body, -1, vrep.simx_opmode_buffer)
                position_history.append(body_pos)
                e, joint_0_pos = vrep.simxGetJointPosition(
                    client_id, joint_0, vrep.simx_opmode_buffer)
                e, joint_1_pos = vrep.simxGetJointPosition(
                    client_id, joint_1, vrep.simx_opmode_buffer)
                joint_pos_history.append([joint_0_pos, joint_1_pos])

            e = vrep.simxSetJointTargetPosition(client_id, joint_1, 0.0,
                                                vrep.simx_opmode_streaming)
            for t in range(8):
                vrep.simxSynchronousTrigger(client_id)
                e, body_pos = vrep.simxGetObjectPosition(
                    client_id, body, -1, vrep.simx_opmode_buffer)
                position_history.append(body_pos)
                e, joint_0_pos = vrep.simxGetJointPosition(
                    client_id, joint_0, vrep.simx_opmode_buffer)
                e, joint_1_pos = vrep.simxGetJointPosition(
                    client_id, joint_1, vrep.simx_opmode_buffer)
                joint_pos_history.append([joint_0_pos, joint_1_pos])

    print "Elapsed time (wall-clock): ", timer.elapsed

    # plot xyz trajectory of the body
    T = len(position_history)
    plt.gca().set_color_cycle('rgb')
    plt.plot(np.arange(T) * 0.05, np.array(position_history))
    plt.legend(['x', 'y', 'z'], loc='best')
    plt.title('Position of the body')
    plt.xlabel('time in simulation [s]')
    plt.ylabel('position [m]')
    plt.grid()
    plt.show()

    # plot joint positions
    plt.figure()
    T = len(joint_pos_history)
    plt.gca().set_color_cycle('rgb')
    plt.plot(np.arange(T) * 0.05, np.array(joint_pos_history))
    plt.legend(['joint_0', 'joint_1'], loc='best')
    plt.title('Position of the joints')
    plt.xlabel('time in simulation [s]')
    plt.ylabel('position [rad]')
    plt.grid()
    plt.show()

    e = vrep.simxStopSimulation(client_id, vrep.simx_opmode_oneshot_wait)
