import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='quadcopter-v0',
    entry_point='quadcopter.envs:QuadcopterEnv',
)

register(
    id='quadcopter-noise-v0',
    entry_point='quadcopter.envs:QuadcopterEnvNoise',
)