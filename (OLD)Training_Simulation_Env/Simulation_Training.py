import numpy as np
from stable_baselines3 import PPO
import torch as th
import cmath

cmath
env_name = "Simulation_Env/3DPos.exe"   # Name of the Unity environment binary to launch

import sys
from gym_unity.envs import UnityToGymWrapper
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
print("Python version:")
print(sys.version)


channel = EngineConfigurationChannel()
unity_env = UnityEnvironment(env_name, side_channels=[channel])
channel.set_configuration_parameters(time_scale = 2)
env = UnityToGymWrapper(unity_env)


from stable_baselines3.common.callbacks import CheckpointCallback, EveryNTimesteps
checkpoint_on_event = CheckpointCallback(save_freq=1, save_path='logs_models/logs_Sim_env/')
event_callback = EveryNTimesteps(n_steps=100000, callback=checkpoint_on_event)


policy_kwargs = dict(activation_fn=th.nn.ReLU,
                     net_arch=[dict(pi=[128, 256,256, 128], vf=[128, 256,256, 128])])


#Training
################################
"""model = PPO("MlpPolicy", env, verbose=2, tensorboard_log="./logs_graphs/PPO_3DSim_2000000_new_4_3/", policy_kwargs=policy_kwargs)
model.learn(total_timesteps=2000000,reset_num_timesteps=True, tb_log_name="1", callback=event_callback)
model.save("trained_models/sim_model_PPO")"""
################################

#Checking Results
#################################
model = PPO.load("trained_models/sim_model_PPO.zip", env)
model.set_env(env)
model.learn(total_timesteps=0)
#################################

obs = env.reset()
while True:
    #action, _states = model.predict(obs)
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    print("Act: " + str(action))
    print("Obs : " + str(obs))
    if dones:
        env.reset()
    env.render()