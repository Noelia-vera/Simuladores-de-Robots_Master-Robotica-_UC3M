#MASTER EN ROBÓTICA Y AUTOMATIZACIÓN
#NOELIA FERNANDEZ TALAVERA
#SIMULADORES DE ROBOTS-GYMNASIUM
#2024
#En este código se proporciona una solución para llegar desde la esquina izquierda superior de un mapa a la esquina inferior derecha
#En el mapa hay una serie de obstáculos que hay que esquivar
#--------------------------------------------------------------------------------------------------------------------------------------------

#!/usr/bin/env python

import gymnasium as gym
import gymnasium_csv
from gymnasium_csv.wrappers import BoxToDiscreteObservation

import numpy as np
import time

"""
# Coordinate Systems for `.csv` and `print(numpy)`

X points down (rows); Y points right (columns); Z would point outwards.

*--> Y (columns)
|
v
X (rows)
"""
UP = 0
UP_RIGHT = 1
RIGHT = 2
DOWN_RIGHT = 3
DOWN = 4
DOWN_LEFT = 5
LEFT = 6
UP_LEFT = 7

SIM_PERIOD_MS = 500.0

env_raw = gym.make('gymnasium_csv-v0',
               render_mode='human',  # "human", "text", None
               inFileStr='../assets/laberinto_p.csv',
               initX=2,
               initY=2,
               goalX=16,
               goalY=10)
env = BoxToDiscreteObservation(env_raw)
observation, info = env.reset()
print("observation: "+str(observation)+", info: "+str(info))
env.render()
time.sleep(0.5)

for i in range(7):
    observation, reward, terminated, truncated, info = env.step(DOWN_RIGHT)
    env.render()
    print("observation: " + str(observation)+", reward: " + str(reward) + ", terminated: " +
          str(terminated) + ", truncated: " + str(truncated) + ", info: " + str(info))
    time.sleep(SIM_PERIOD_MS/1000.0)

for i in range(6):
    observation, reward, terminated, truncated, info = env.step(DOWN)
    env.render()
    print("observation: " + str(observation)+", reward: " + str(reward) + ", terminated: " +
          str(terminated) + ", truncated: " + str(truncated) + ", info: " + str(info))
    time.sleep(SIM_PERIOD_MS/1000.0)
for i in range(1):
    observation, reward, terminated, truncated, info = env.step(DOWN_RIGHT)
    env.render()
    print("observation: " + str(observation)+", reward: " + str(reward) + ", terminated: " +
          str(terminated) + ", truncated: " + str(truncated) + ", info: " + str(info))
    time.sleep(SIM_PERIOD_MS/1000.0)