# -*- coding: utf-8 -*-
"""
Created on Mon Aug 10 20:38:01 2015

@author: sakurai
"""

import copy
import numpy as np
import matplotlib.pyplot as plt
import vrep
import contexttimer
import chainer.functions as F
from chainer import Variable, FunctionSet, optimizers
from environment import Robot


class MLP(FunctionSet):
    def __init__(self, input_dim, output_dim, h1_dim=20, h2_dim=20):
        sqrt2 = np.sqrt(2)
        super(MLP, self).__init__(
            fc1=F.Linear(input_dim, h1_dim, wscale=1),
            fc2=F.Linear(h1_dim, h2_dim, wscale=sqrt2),
            fc3=F.Linear(h2_dim, output_dim, wscale=sqrt2),
        )

    def forward(self, x_data, train=False):
        x_data = np.atleast_2d(x_data)
        x = Variable(x_data, volatile=not train)
        h = F.relu(self.fc1(x))
        h = F.relu(self.fc2(h))
        y = self.fc3(h)
        return y


class Memory(object):
    def __init__(self, max_memory_size, state_dims):
        self.max_memory_size = max_memory_size
        self.states = np.zeros((max_memory_size, state_dims), dtype=np.float32)
        self.actions = np.zeros(max_memory_size, dtype=np.int32)
        self.rewards = np.zeros(max_memory_size, dtype=np.float32)
        self.next_states = np.zeros_like(self.states)
        self.memory_size = 0
        self.index = 0

    def append(self, states, actions, rewards, next_states):
        actions, rewards = np.atleast_1d(actions), np.atleast_1d(rewards)
        states, next_states = np.atleast_2d(states), np.atleast_2d(next_states)
        num_examples = len(states)
        assert num_examples == len(actions) == len(rewards) == len(next_states)

        if self.memory_size < self.max_memory_size:
            self.memory_size += num_examples

        start, end = self.index, self.index + num_examples
        if end >= self.max_memory_size:
            start, end = 0, num_examples
        self.index = end

        self.states[start:end] = states
        self.actions[start:end] = actions
        self.rewards[start:end] = rewards
        self.next_states[start:end] = next_states

    def get_data(self):
        return (self.states[:self.memory_size],
                self.actions[:self.memory_size],
                self.rewards[:self.memory_size],
                self.next_states[:self.memory_size])

    def get_random_batch(self, batch_size):
        states, actions, rewards, next_states = self.get_data()
        num_examples = len(states)
        if batch_size > num_examples:
            batch_size = num_examples
        indexes = np.random.choice(num_examples, batch_size, replace=True)
        return (states[indexes], actions[indexes],
                rewards[indexes], next_states[indexes])


class NFQAgent(object):
    def __init__(self, robot, gamma=0.9, exploration_episodes=100,
                 epsilon_initial=1.0, epsilon_final=0.1,
                 memory_size=200000, num_h1=20, num_h2=20,
                 learning_rate=0.0001, num_epochs=5, minibatch_size=100
                 ):
        self.robot = robot
        self.num_actions_0 = 3
        self.num_actions_1 = 3
        self.num_actions = self.num_actions_0 * self.num_actions_1
        self.angles_0 = np.linspace(0, np.pi/2, self.num_actions_0)
        self.angles_1 = np.linspace(0, np.pi/2, self.num_actions_1)
        # look-up table from action to angles
        self.angles_lut = np.array(np.meshgrid(self.angles_1, self.angles_0,
                                   indexing='ij')).reshape(2, -1).T

        self.state_dims = 6

        self.gamma = gamma      # discount factor
        self.epsilon_initial = epsilon_initial
        self.epsilon_final = epsilon_final
        self.epsilon = epsilon_initial  # epsilon-greedy rate
        self.exploration_episodes = exploration_episodes

        self.memory_size = memory_size
        self.memory = Memory(self.memory_size, self.state_dims)
        self.num_h1, self.num_h2 = num_h1, num_h2
        self.lr = learning_rate      # learning rate
        self.num_epochs = num_epochs
        self.minibatch_size = minibatch_size

        self.mlp = MLP(self.state_dims, self.num_actions, num_h1, num_h2)
        self.target_mlp = copy.deepcopy(self.mlp)
        self.optimizer = optimizers.MomentumSGD(self.lr)
        self.optimizer = optimizers.RMSprop(self.lr)
        self.optimizer = optimizers.RMSpropGraves(self.lr)
        self.optimizer.setup(self.mlp.collect_parameters())

        self.num_episodes = 0

    def choose_action(self, state):
        if np.random.uniform() < self.epsilon:
            action = np.random.choice(self.num_actions)
            print "R{}".format(action),
        else:
            q = self.mlp.forward(state, train=False).data.ravel()
            action = np.argmax(q)
            print action, q[action],
        return action

    def do_action(self, action):
        angles = self.angles_lut[action]
        self.robot.set_joint_angles(angles)
        self.robot.proceed_simulation()

    def observe_state(self):
        """The state is defined as
        (angle_0, angle_1, body_z, angle_inc_0, angle_inc_1, body_inc_z),
        where body_z corresponds to up-down axis
        """
        current_angles = self.robot.get_joint_angles()
        last_angles = self.state[:2]
        delta_angle = current_angles - last_angles

        current_position = self.robot.get_body_position()
        current_z = np.atleast_1d(current_position[2])
        last_z = self.state[2]
        delta_z = current_z - last_z

        state = np.concatenate((current_angles, current_z,
                                delta_angle, delta_z))
        return state

    def play_one_step(self):
        action = self.choose_action(self.state)
        self.do_action(action)

        state_new = self.observe_state()

        position_new = self.robot.get_body_position()
        x_forward = position_new[0] - self.position[0]
        reward = x_forward - 0.001

        self.add_memory(self.state, action, reward, state_new)
        loss = self.update_q_function()

        self.state = state_new
        self.position = position_new

        return loss

    def increment_episode(self):
        self.num_episodes += 1

    def update_epsilon(self):
        initial, final = self.epsilon_initial, self.epsilon_final
        if self.epsilon > final:
#            difference = initial - final
#            self.epsilon -= difference / float(self.exploration_episodes)
            difference = initial / float(final)
            self.epsilon /= difference ** (1.0 / self.exploration_episodes)
        else:
            self.epsilon = final

    def add_memory(self, state, action, reward, state_new):
        self.memory.append(state, action, reward, state_new)

    def initialize_episode(self):
        self.robot.restart_simulation()
        self.robot.initialize_pose()
        self.position = self.robot.get_body_position()
        self.state = np.zeros(self.state_dims, dtype=np.float32)
        self.state = self.observe_state()

    def update_q_function(self):
        minibatch = self.memory.get_random_batch(self.minibatch_size)
        states, actions, rewards, next_states = minibatch
        num_examples = len(states)

        self.optimizer.zero_grads()
        y = self.mlp.forward(states, train=True)

        # make targets
        next_q = self.target_mlp.forward(next_states, train=False).data
        optimal_next_q = np.max(next_q, axis=1)
        bellman_targets = rewards + self.gamma * optimal_next_q
        t_data = y.data.copy()
        t_data[np.arange(num_examples), actions] = bellman_targets
        t = Variable(t_data)

        loss = F.mean_squared_error(y, t)
        loss.backward()
        self.optimizer.update()

        return loss.data

    def update_target_q_function(self):
        self.target_mlp = copy.deepcopy(self.mlp)

    def show_q_function(self):
        """show mappings from state space to reward for each action"""
        x = np.linspace(0, np.pi/2, 100, endpoint=True)
        y = np.linspace(0, np.pi/2, 100, endpoint=True)
        X, Y = np.meshgrid(x, y)
        states = np.vstack((X.ravel(), Y.ravel())).T.astype(np.float32)
        q = self.mlp.forward(states, False).data.T.reshape(-1, 100, 100)
        vmin, vmax = q.min(), q.max()
        fig, axs = plt.subplots(3, 3)
        for q_a, ax in zip(q, axs.ravel()):
            ax.matshow(q_a, vmin=vmin, vmax=vmax)
            ax.set_xticks([0, 50, 100])
            ax.set_xticklabels([0, 45, 90])
            ax.set_yticks([0, 50, 100])
            ax.set_yticklabels([0, 45, 90])
        plt.tight_layout()
        plt.show()
        plt.draw()


def plot(body_trajectory, joints_trajectory, return_history, loss_history):
    plt.gcf().set_size_inches(8, 4)
    T = len(body_trajectory)
    ax1 = plt.subplot(221)
    ax2 = plt.subplot(222)
    ax3 = plt.subplot(223)
    ax4 = plt.subplot(224)
    # plot an xyz trajectory of the body
    ax1.grid()
    ax1.set_color_cycle('rgb')
    ax1.plot(np.arange(T) * 0.05, np.array(body_trajectory))
    ax1.set_title('Position of the body')
    ax1.set_ylabel('position [m]')
    ax1.legend(['x', 'y', 'z'], loc='best')

    # plot a history of returns of each episode
    ax2.grid()
    ax2.plot(return_history)
    ax2.set_title('Returns (total rewards) of each episode')
    ax2.set_xlabel('episode')
    ax2.set_ylabel('position [m]')

    # plot a trajectory of angles of the joints
    ax3.grid()
    ax3.set_color_cycle('rg')
    ax3.plot(np.arange(T) * 0.05, np.array(joints_trajectory))
    ax3.set_title('Angle of the joints')
    ax3.set_xlabel('time in simulation [s]')
    ax3.set_ylabel('angle [rad]')
    ax3.set_ylim([0, np.pi/2])
    ax3.legend(['joint_0', 'joint_1'], loc='best')

    # plot a learning curve of the mlp
    ax4.grid()
    ax4.plot(loss_history)
    ax4.set_title('Learning curve')
    ax4.set_xlabel('batches')
    ax4.set_ylabel('loss')

    plt.tight_layout()
    plt.show()
    plt.draw()

if __name__ == '__main__':
    try:
        client_id
    except NameError:
        client_id = -1
    e = vrep.simxStopSimulation(client_id, vrep.simx_opmode_oneshot_wait)
    vrep.simxFinish(-1)
    client_id = vrep.simxStart('127.0.0.1', 19998, True, True, 5000, 5)
    assert client_id != -1, 'Failed connecting to remote API server'

    # print ping time
    sec, msec = vrep.simxGetPingTime(client_id)
    print "Ping time: %f" % (sec + msec / 1000.0)

    robot = Robot(client_id)
    agent = NFQAgent(robot, gamma=0.9, exploration_episodes=1000,
                     epsilon_initial=1.0, epsilon_final=0.1,
                     memory_size=200000, num_h1=20, num_h2=20,
                     learning_rate=0.0001, num_epochs=1, minibatch_size=50)

    num_episodes = 2000
    len_episode = 100
    return_history = []
    loss_history = []
    try:
        for episode in range(num_episodes):
            print "start simulation # %d" % episode

            # run an episode of simulation
            with contexttimer.Timer() as timer:
                agent.initialize_episode()
                body_trajectory = [robot.get_body_position()]
                joints_trajectory = [robot.get_joint_angles()]

                for t in range(len_episode):
                    loss = agent.play_one_step()

                    loss_history.append(loss)
                    body_trajectory.append(robot.get_body_position())
                    joints_trajectory.append(robot.get_joint_angles())
            time_for_episode = timer.elapsed

            position = body_trajectory[-1]
            return_history.append(position[0])

            # update target Q-function
            if (episode + 1) % 25 == 0:
                agent.update_target_q_function()
                loss_history = []

            agent.update_epsilon()
            agent.increment_episode()

            plot(body_trajectory, joints_trajectory,
                 return_history, loss_history)
            print
            print "Body position: ", position
            print "Time to play this episode (wall-clock): ", time_for_episode
            print "Current epsilon", agent.epsilon
            print "|W|", [np.linalg.norm(p) for p in agent.mlp.parameters[::2]]
            print

    except KeyboardInterrupt:
        print "Terminated by `Ctrl+c` !!!!!!!!!!"

    plt.grid()
    plt.plot(return_history)
    plt.title('Return (total reward in a episode)')
    plt.xlabel('episode')
    plt.ylabel('position [m]')
    plt.show()

    e = vrep.simxStopSimulation(client_id, vrep.simx_opmode_oneshot_wait)
