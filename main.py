# import gym
# from baselines import deepq
# import quadcopter
#
#
# def callback(lcl, glb):
#     # stop training if reward exceeds 199
#     is_solved = lcl['t'] > 100 and sum(lcl['episode_rewards'][-101:-1]) / 100 >= 199
#     return is_solved
#
#
# def main():
#     env = gym.make("quadcopter-v0")
#     model = deepq.models.mlp([16, 12])
#     act = deepq.learn(
#         env,
#         q_func=model,
#         lr=1e-3,
#         max_timesteps=100000,
#         buffer_size=100000,
#         exploration_fraction=0.1,
#         exploration_final_eps=0.02,
#         print_freq=10,
#         callback=callback
#     )
#     print("Saving model to balance.pkl")
#     act.save("balance.pkl")
#
#
# if __name__ == '__main__':
#     main()
import gym
import quadcopter

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common import make_vec_env
from stable_baselines import PPO2


def train():
    # multiprocess environment
    env = gym.make("quadcopter-v0")
    model = PPO2(MlpPolicy, env, verbose=1)
    model.learn(total_timesteps=20000000)
    model.save("ppo2_quadcopter")

    del model  # remove to demonstrate saving and loading


def simulate():
    env = gym.make("quadcopter-v0", render=True)
    model = PPO2.load("ppo2_quadcopter")
    obs = env.reset()
    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        env.render()


# train()
simulate()

