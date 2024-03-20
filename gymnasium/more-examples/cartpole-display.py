#MASTER EN ROBÓTICA Y AUTOMATIZACIÓN
#NOELIA FERNANDEZ TALAVERA
#SIMULADORES DE ROBOTS-GYMNASIUM
#2024
#En esta práctica se implementa una técnica para mantener un pendulo invertido en equilibrio
#--------------------------------------------------------------------------------------------------------------------------------------------


#!/usr/bin/env python

import gymnasium as gym
import time
import numpy as np
import matplotlib.pyplot as plt

def run():
    env = gym.make('CartPole-v1', render_mode="human")

    pos_space = np.linspace(-2.4, 2.4, 10)
    vel_space = np.linspace(-4, 4, 10)
    ang_space = np.linspace(-.2095, .2095, 10)
    ang_vel_space = np.linspace(-4, 4, 10)

    q = np.zeros((len(pos_space)+1, len(vel_space)+1, len(ang_space)+1, len(ang_vel_space)+1, env.action_space.n))

    learning_rate_a = 0.1
    discount_factor_g = 0.99
    epsilon = 1
    epsilon_decay_rate = 0.00001
    rng = np.random.default_rng()

    rewards_per_episode = []

    i = 0

    while(True):
        state = env.reset()[0]
        state_p = np.digitize(state[0], pos_space)
        state_v = np.digitize(state[1], vel_space)
        state_a = np.digitize(state[2], ang_space)
        state_av = np.digitize(state[3], ang_vel_space)

        terminated = False
        rewards = 0

        while(not terminated and rewards < 10000):
            if rng.random() < epsilon:
                action = env.action_space.sample()
            else:
                action = np.argmax(q[state_p, state_v, state_a, state_av, :])

            new_state, reward, terminated, _, _ = env.step(action)
            new_state_p = np.digitize(new_state[0], pos_space)
            new_state_v = np.digitize(new_state[1], vel_space)
            new_state_a = np.digitize(new_state[2], ang_space)
            new_state_av = np.digitize(new_state[3], ang_vel_space)

            q[state_p, state_v, state_a, state_av, action] = q[state_p, state_v, state_a, state_av, action] + learning_rate_a * (
                reward + discount_factor_g*np.max(q[new_state_p, new_state_v, new_state_a, new_state_av,:]) - q[state_p, state_v, state_a, state_av, action]
            )

            state = new_state
            state_p = new_state_p
            state_v = new_state_v
            state_a = new_state_a
            state_av = new_state_av

            rewards += reward

            if rewards % 100 == 0:
                print(f'Episode: {i}  Rewards: {rewards}')

        rewards_per_episode.append(rewards)
        mean_rewards = np.mean(rewards_per_episode[max(0, len(rewards_per_episode)-100):])

        if i % 100 == 0:
            print(f'Episode: {i} {rewards}  Epsilon: {epsilon:0.2f}  Mean Rewards {mean_rewards:0.1f}')

        if mean_rewards > 1000:
            break

        epsilon = max(epsilon - epsilon_decay_rate, 0)

        i += 1

    env.close()

    mean_rewards = []
    for t in range(i):
        mean_rewards.append(np.mean(rewards_per_episode[max(0, t-100):(t+1)]))
    plt.plot(mean_rewards)
    plt.savefig(f'cartpole.png')

if __name__ == '__main__':
    run()
