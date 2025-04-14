import numpy as np
from stable_baselines3 import PPO
import torch as th
import cmath
import torch

#SEED = 300
#np.random.seed(SEED)
#torch.manual_seed(SEED)

cmath
env_name = "SimulationEnv/3DPos.exe"   # Name of the Unity environment binary to launch

import sys
from gym_unity.envs import UnityToGymWrapper
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
print("Python version:")
print(sys.version)

# check Python version
if (sys.version_info[0] < 3):
    raise Exception("ERROR: ML-Agents Toolkit (v0.3 onwards) requires Python 3")


channel = EngineConfigurationChannel()
unity_env = UnityEnvironment(env_name, side_channels=[channel])
channel.set_configuration_parameters(time_scale = 2)
env = UnityToGymWrapper(unity_env)


from stable_baselines3.common.callbacks import CheckpointCallback, EveryNTimesteps
checkpoint_on_event = CheckpointCallback(save_freq=1, save_path='logs_models/PPO_3DSim_2M_random_4/')
event_callback = EveryNTimesteps(n_steps=100000, callback=checkpoint_on_event)


policy_kwargs = dict(activation_fn=th.nn.ReLU,
                     net_arch=[dict(pi=[128, 256,256, 128], vf=[128, 256,256, 128])])


# For training
"""model = PPO("MlpPolicy", env, verbose=2, tensorboard_log="./logs_graphs/PPO_3DSim_2M_random_4/", policy_kwargs=policy_kwargs, learning_rate=1e-4)
model.learn(total_timesteps=2000000,reset_num_timesteps=True, tb_log_name="1", callback=event_callback)
model.save("trained_models/PPO_3DSim_2M_random_4")"""


# For testing
model = PPO("MlpPolicy", env, policy_kwargs=policy_kwargs)
model = PPO.load("logs_models/PPO_3DSim_2M_random_1/rl_model_2000000_steps.zip", env)
model.set_env(env)
model.learn(total_timesteps=0)

obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    print("Act: " + str(action))
    print("Obs : " + str(obs))
    if dones:
        env.reset()
    env.render()