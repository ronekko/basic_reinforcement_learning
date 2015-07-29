# -*- coding: utf-8 -*-
"""
Created on Sat Jul 18 15:44:19 2015

@author: ryuhei
"""

import time
import numpy as np
import vrep

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

#    for i in range(10):
#        angle_0, angle_1 = np.random.uniform(-1, 1, 2)
#        e = vrep.simxSetJointTargetPosition(client_id, joint_0, angle_0,
#                                            vrep.simx_opmode_streaming)
#        e = vrep.simxSetJointTargetPosition(client_id, joint_1, angle_0,
#                                            vrep.simx_opmode_streaming)
#        time.sleep(1)

    for i in range(4):
        e = vrep.simxSetJointTargetPosition(client_id, joint_0, 0.5,
                                            vrep.simx_opmode_streaming)
        for t in range(3):
            vrep.simxSynchronousTrigger(client_id)

        e = vrep.simxSetJointTargetPosition(client_id, joint_1, 2.5,
                                            vrep.simx_opmode_streaming)
        for t in range(10):
            vrep.simxSynchronousTrigger(client_id)

        e = vrep.simxSetJointTargetPosition(client_id, joint_0, 0.0,
                                            vrep.simx_opmode_streaming)
        for t in range(2):
            vrep.simxSynchronousTrigger(client_id)

        e = vrep.simxSetJointTargetPosition(client_id, joint_1, 0.0,
                                            vrep.simx_opmode_streaming)
        for t in range(8):
            vrep.simxSynchronousTrigger(client_id)

    # Get absolute position of the body with specifying -1
    e, body_pos = vrep.simxGetObjectPosition(client_id, body, -1,
                                             vrep.simx_opmode_buffer)
    print body_pos

    e = vrep.simxStopSimulation(client_id, vrep.simx_opmode_oneshot_wait)
