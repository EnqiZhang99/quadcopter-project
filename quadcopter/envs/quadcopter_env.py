import os
import math
import numpy as np

import gym
from gym import spaces
from gym.utils import seeding

import pybullet as p
import pybullet_data

from quadcopter.envs.parcour import obstacle

color = [0.4, 0.4, 0.4, 1]
contactColor = [1., 0., 0., 1]
directions = [[-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], [0, 0, -1], [0, 0, 1]]


class QuadcopterEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self, render=False):
        self.observation = []

        self.action_space = spaces.Box(0, 1, (4,), dtype=np.float32)
        self.observation_space = spaces.Box(
            np.array([10.0, 10.0, 10.0, 4.0, 4.0, 8.0, 1.0, 1.0, 1.0, 1.0]),
            np.array([-10.0, -10.0, -10.0, -4.0, -4.0, 0.0, 0.0, 0.0, 0.0, 0.0]))


        if render:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)  # non-graphical version

        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF

        self.seed()

        # paramId = p.addUserDebugParameter("My Param", 0, 100, 50)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        action = np.clip(action, 0.0, 1.0)
        assert action.shape == (4,)
        self.simulate_quadrotor(action * 10)
        p.stepSimulation()

        self.observation = self.compute_observation()
        reward = self.compute_reward()
        done = self.compute_done()

        self.envStepCounter += 1

        return np.array(self.observation), reward, done, {}

    def reset(self):
        p.resetSimulation()
        p.setTimeStep(0.01)
        p.setGravity(0, 0, -10)
        path = os.path.abspath(os.path.dirname(__file__))
        self.copter = p.loadURDF(os.path.join(path, "quadcopter.urdf"), [-3, -3, 2.], [0, 0, 0, 1])
        p.changeDynamics(self.copter, -1, linearDamping=0.9)
        p.loadURDF("plane.urdf")

        obstacle()

        self.envStepCounter = 0

        # you *have* to compute and return the observation from reset()
        self.observation = self.compute_observation()
        return np.array(self.observation)

    def simulate_quadrotor(self, action):
        force1 = [0., 0., action[0]]
        force2 = [0., 0., action[1]]
        force3 = [0., 0., action[2]]
        force4 = [0., 0., action[3]]

        p.applyExternalForce(self.copter, -1, force1, [0, 0.25, 0], flags=p.LINK_FRAME)
        p.applyExternalForce(self.copter, -1, force2, [0, -0.25, 0], flags=p.LINK_FRAME)
        p.applyExternalForce(self.copter, -1, force3, [0.25, 0, 0], flags=p.LINK_FRAME)
        p.applyExternalForce(self.copter, -1, force4, [-0.25, 0, 0], flags=p.LINK_FRAME)

    def compute_observation(self):
        pos, orn = p.getBasePositionAndOrientation(self.copter)
        linear, _ = p.getBaseVelocity(self.copter)
        norm_linear = [l / 7 for l in linear]
        norm_pos = [pos[0] / 4, pos[1] / 4, pos[2] / 4 - 1]
        observation = [*linear, *pos, *orn]
        return observation

    def compute_reward(self):
        pos, _ = p.getBasePositionAndOrientation(self.copter)
        return (10 - euclidean_distance(pos, (3, 3, 7))) * 100

    def compute_done(self):
        pos, _ = p.getBasePositionAndOrientation(self.copter)
        reward = 10 - euclidean_distance(pos, (3, 3, 7))
        return p.getContactPoints(self.copter) or reward > 9.9

    def render(self, mode='human', close=False):
        pass


def euclidean_distance(x, y):
    return math.sqrt(sum([(a - b) ** 2 for a, b in zip(x, y)]))
