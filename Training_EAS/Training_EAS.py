import os
os.environ['KMP_DUPLICATE_LIB_OK']='True'

from stable_baselines3 import PPO
from EAS_env import EAS_env
import keyboard
import torch as th


env = EAS_env()

# Examine environment parameters
print("OBS Shape: " + str(env.observation_space.shape))
print("Action Shape: " + str(env.action_space.shape))
print(str(env.action_space.sample()))

env.reset()
from stable_baselines3.common.callbacks import BaseCallback
class reset_currents_callback(BaseCallback):
    def __init__(self, verbose: int = 1):
        super(reset_currents_callback, self).__init__(verbose)

    def _on_step(self) -> bool:
        return True
    def _on_rollout_end(self) -> None:
        self.training_env.render()
        print("IN CALLBACK")


from stable_baselines3.common.callbacks import CheckpointCallback, EveryNTimesteps
checkpoint_on_event = CheckpointCallback(save_freq=1, save_path="logs_models/PPO/1")
event_callback = EveryNTimesteps(n_steps=10000, callback=checkpoint_on_event)
rcc = reset_currents_callback()
from stable_baselines3.common.callbacks import CallbackList
callback = CallbackList([event_callback, rcc])

policy_kwargs = dict(activation_fn=th.nn.ReLU,
                     net_arch=[dict(pi=[128, 256, 256 ,128], vf=[128, 256, 256 ,128])])


model = PPO.load("trained_models/sim_model_PPO.zip", env)

model.verbose=2; model.tensorboard_log="logs_graphs/PPO_MTS_4000000_3D_1_bws_2"; model.policy_kwargs=policy_kwargs
model.env.reset()
model.learn(total_timesteps=2000000, reset_num_timesteps=False, tb_log_name="1", callback=callback)
model.save("Trained_model_EAS")


def keyboardCont(a):
    if keyboard.is_pressed('left'):
        print("Left")
        return [0, 0 ,0 , 1]

    elif keyboard.is_pressed('right'):
        print("Right")
        return [0, 1, 0, 0]

    elif keyboard.is_pressed('down'):
        print("Down")
        return [1, 0, 0, 0]

    elif keyboard.is_pressed('up'):
        print("Up")
        return [0, 0, 1, 0]

    elif keyboard.is_pressed('p'):
        print("Stop")
        return [0, 0, 0, 0]
    else:
        return a

obs = env.reset()

while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    print("Act: " + str(action))
    print("Obs : " + str(obs))
    if dones:
        env.reset()



