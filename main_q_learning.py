# -*- coding: utf-8 -*-
"""
Created on Fri Jul 31 18:14:03 2015

@author: sakurai
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import vrep
import contexttimer

# define actions
num_actions_0 = 3
num_actions_1 = 3
num_actions = num_actions_0 * num_actions_1
angles_0 = np.linspace(0, np.pi/2, num_actions_0)
angles_1 = np.linspace(0, np.pi/2, num_actions_0)
angles_lut = np.array(np.meshgrid(angles_1, angles_0,
                                  indexing='ij')).reshape(2, -1).T

# define states
# state = 0 if x < pi/2K, 1 if pi/2K <= x < 2pi/2K, ..., K-1 if (K-1)pi/2K <= x
num_states_0 = 5  # angle of joint 0
num_states_1 = 5  # angle of joint 1
num_states = num_states_0 * num_states_1
state_bins = [np.linspace(0, np.pi/2, num_states_0, endpoint=False)[1:],
              np.linspace(0, np.pi/2, num_states_1, endpoint=False)[1:]]

# define Q-table
q_table = np.zeros((num_states, num_actions), dtype=np.float) + 0.5


def choose_action(state):
    epsilon = 0.01
    if np.random.uniform() < epsilon:
        action = np.random.choice(num_actions)
    else:
        action = np.argmax(q_table[state])
    return action


def calc_state(angles):
    state_0 = np.digitize([angles[0]], state_bins[0])[0]
    state_1 = np.digitize([angles[1]], state_bins[1])[0]
    state_new = state_0 * num_states_1 + state_1
    return state_new


def act(joints, state, body_pos):
    alpha = 0.1  # learning rate
    gamma = 0.9  # discount factor

    action = choose_action(state)
    angles = angles_lut[action]  # look-up table from action to angles

    for joint, angle in zip(joints, angles):
        if not np.isnan(angle):
            e = vrep.simxSetJointTargetPosition(client_id, joint, angle,
                                                vrep.simx_opmode_streaming)
    vrep.simxSynchronousTrigger(client_id)

    angles_new = []
    for joint in joints:
        e, angle_new = vrep.simxGetJointPosition(client_id, joint,
                                                 vrep.simx_opmode_buffer)
        angles_new.append(angle_new)

    e, body_pos_new = vrep.simxGetObjectPosition(
        client_id, body, -1, vrep.simx_opmode_buffer)
    x_forward = body_pos_new[0] - body_pos[0]
    reward = x_forward - 0.001
    body_pos = body_pos_new

    state_new = calc_state(angles_new)

    q_sa = q_table[state, action]
    td_error = reward + gamma * np.max(q_table[state_new]) - q_sa
    q_table[state, action] = q_sa + alpha * td_error

    return (state_new, body_pos_new)


def restart_simulation(client_id, joints):
    e = vrep.simxStopSimulation(client_id, vrep.simx_opmode_oneshot_wait)
    time.sleep(0.5)
    e = vrep.simxStartSimulation(client_id, vrep.simx_opmode_oneshot_wait)
    e = vrep.simxSetJointTargetPosition(client_id, joints[0], 0,
                                        vrep.simx_opmode_streaming)
    e = vrep.simxSetJointTargetPosition(client_id, joints[1], 0,
                                        vrep.simx_opmode_streaming)
    for t in range(10):
        vrep.simxSynchronousTrigger(client_id)
    e, body_pos = vrep.simxGetObjectPosition(client_id, body, -1,
                                             vrep.simx_opmode_streaming)
    e, angle_0 = vrep.simxGetJointPosition(client_id, joints[0],
                                           vrep.simx_opmode_streaming)
    e, angle_1 = vrep.simxGetJointPosition(client_id, joints[1],
                                           vrep.simx_opmode_streaming)
    state = calc_state([angle_0, angle_1])
    return (state, body_pos)


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
    joints = [joint_0, joint_1]

    # Set the joint angles to the default position (0, 0)
    e = vrep.simxSetJointTargetPosition(client_id, joints[0], 0,
                                        vrep.simx_opmode_streaming)
    e = vrep.simxSetJointTargetPosition(client_id, joints[1], 0,
                                        vrep.simx_opmode_streaming)
    state = 0

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
    e, joint_0_pos = vrep.simxGetJointPosition(client_id, joints[0],
                                               vrep.simx_opmode_streaming)
    e, joint_1_pos = vrep.simxGetJointPosition(client_id, joints[1],
                                               vrep.simx_opmode_streaming)
    joint_pos_history.append([joint_0_pos, joint_1_pos])

    num_episodes = 400
    len_episode = 100
    try:
        for episode in range(num_episodes):
            print "start simulation # %d" % episode
            with contexttimer.Timer() as timer:
                state, body_pos = restart_simulation(client_id, joints)

                for t in range(len_episode):
                    state, body_pos = act(joints, state, body_pos)
                    print state,

                    position_history.append(body_pos)
                    e, joint_0_pos = vrep.simxGetJointPosition(
                        client_id, joints[0], vrep.simx_opmode_buffer)
                    e, joint_1_pos = vrep.simxGetJointPosition(
                        client_id, joints[1], vrep.simx_opmode_buffer)
                    joint_pos_history.append([joint_0_pos, joint_1_pos])
                print
            print "Body position: ", body_pos
            print "Elapsed time (wall-clock): ", timer.elapsed
            print
    except KeyboardInterrupt:
        print "Terminated by `Ctrl+c` !!!!!!!!!!"

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
