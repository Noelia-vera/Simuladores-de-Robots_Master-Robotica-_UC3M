#!/usr/bin/env python

"""
pip install gymnasium[box2d]
"""

import gymnasium as gym
import time

env = gym.make('LunarLander-v2', render_mode="human")

print("env.observation_space:", env.observation_space)
print("env.action_space:", env.action_space)

observation, info = env.reset()
print("observation:", observation)
print("info:", info)

env.render()
time.sleep(1)
