#!/usr/bin/env python

"""
# gymnasium[mujoco]==0.27.1 installs mujoco==2.3.2
pip install gymnasium[mujoco]
"""

import gymnasium as gym
import time

env = gym.make('Humanoid-v4', render_mode="human")

print("env.observation_space:", env.observation_space)
print("env.action_space:", env.action_space)

observation, info = env.reset()
print("observation:", observation)
print("info:", info)

env.render()
time.sleep(1)
