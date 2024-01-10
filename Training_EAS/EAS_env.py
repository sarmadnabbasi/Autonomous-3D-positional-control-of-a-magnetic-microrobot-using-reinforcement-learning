import gym
from gym import spaces
from utils.EAS_current_control import EAS_current_control
from utils.tracking.pos_track_3D_prec import pos_track_3D_prec

import cv2
import keyboard
import numpy as np
from time import sleep
import random

class EAS_env(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.con1 = EAS_current_control("COM4", 115200)
        self.reward = 0
        self.total_reward = 0
        self.render_cam1 =None
        self.render_cam2 = None
        self.countChange =3
        self.totalSteps =0
        self.maxSteps =100 #Chage for Different steps
        self.action_prev =[0, 0, 0, 0, 0, 0, 0, 0]

        self.max_roi = 1
        self.th = 0.1 #Change for different steps
        self.oob = True
        self.max_steps_reach = True
        # Actions Space
        self.min_action = -1.0
        self.max_action = 1.0
        self.current_mult = 7

        self.action_space = spaces.Box(
            low=self.min_action,
            high=self.max_action,
            shape=(8,),
            dtype=np.float32
        )

        self.observation_space = spaces.Box(
            low=np.array([-np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf]),
            high=np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]),
            shape=(6,),
            dtype=np.float32
        )

        self.pos_track = pos_track_3D_prec()
        self.posAgentxyz = np.array([0, 0, 0])
        self.prev_posAgentxyz=np.array([0,0,0])
        self.posTargetxyz = np.array([0, 0, 0])
        self.target_th = 0.1
        sleep(2)

    def getPositions(self):

        img1 = self.pos_track.get_posXY(custom_roi=False, roi=[200, 0, 800, 800], visualize=False, normalize_output=True,
                                 debug_ip=False)
        img2 = self.pos_track.get_posZ(custom_roi=False, roi=[200, 0, 800, 800], visualize=False, normalize_output=True,
                                debug_ip=False)

        tempxyz = np.asarray(self.pos_track.pos_3D)
        tempxyz[2] = (tempxyz[2] + 1) / 2

        if np.linalg.norm(tempxyz - self.prev_posAgentxyz) >0:
            self.posAgentxyz = tempxyz
            self.prev_posAgentxyz = tempxyz

        else:
            self.posAgentxyz=self.prev_posAgentxyz

        posTargetxyz_pixels =np.array([0,0,0])
        posTargetxyz_pixels[0] = 400 * (self.posTargetxyz[0] + 1)
        posTargetxyz_pixels[1] = 800 - (400 * (self.posTargetxyz[1] + 1))
        posTargetxyz_pixels[2] = 800 - 800 * (self.posTargetxyz[2])

        posAgentxyz_pixels = np.array([0,0,0])
        posAgentxyz_pixels[0] = 400 * (self.posAgentxyz[0] + 1)
        posAgentxyz_pixels[1] = 800 - (400 * (self.posAgentxyz[1] + 1))
        posAgentxyz_pixels[2] = 800 - (400 * ((2*self.posAgentxyz[2]-1) + 1))

        img1 = cv2.circle(img1, (int(posAgentxyz_pixels[0]), int(posAgentxyz_pixels[1])), 10, (255, 0, 0), -1)
        img2 = cv2.circle(img2, (int(posAgentxyz_pixels[0]), int(posAgentxyz_pixels[2])), 10, (255, 0, 0), -1)


        img1 = cv2.circle(img1, (posTargetxyz_pixels[0], posTargetxyz_pixels[1]), 10, (0, 0, 255), -1)
        img2 = cv2.circle(img2, (posTargetxyz_pixels[0], posTargetxyz_pixels[2]), 10, (0, 0, 255), -1)

        self.render_cam1 = img1
        self.render_cam2 = img2

        concObs = np.concatenate((self.posAgentxyz, self.posTargetxyz), axis=0)

        return concObs


    def _next_observation(self):
        concObs = self.getPositions()
        self.show_renders()
        concObs = concObs.astype('float32')
        return concObs

    def _take_action(self, action):
        action = self.current_mult * abs(action)
        action = np.round(action, 2)
        self.con1.sendPWM(str(action[0])+" "+str(action[1])+" "+str(action[2])+" "+str(action[3])+" "+str(action[4])+" "+str(action[5])+" "+str(action[6])+" "+str(action[7])+"\n") #Fix

    def step(self, action):
        self.totalSteps = self.totalSteps + 1
        self._take_action(action)
        obs = self._next_observation()

        posAgent = obs[0:3].copy()
        posTarget = obs[3:6].copy()
        posAgent[2] = 2 * posAgent[2] - 1
        posTarget[2] = 2 * posTarget[2] - 1

        distance = np.linalg.norm(posAgent - posTarget)

        done = False

        if (abs(posAgent[0]) > self.max_roi or abs(posAgent[1]) > self.max_roi or abs(obs[2]) > self.max_roi):
            self.con1.sendStop()
            print("OOB")
            self.reward = -5
            self.oob = True
            done = True

        elif self.totalSteps > self.maxSteps:
            self.max_steps_reach = True
            print("Max Steps")
            self.reward = - distance
            self.totalSteps = 0
            self.con1.sendStop()
            done = True

        self.total_reward = self.total_reward + self.reward

        if keyboard.is_pressed('p'):
            self.reset_mid_episode()

        if keyboard.is_pressed('o'):
            self.reset_mid_episode()

        return obs, self.reward, done, {}

    def reset(self):
        print("Total Reward : {}".format(self.total_reward))
        self.totalSteps =0
        self.total_reward = 0
        obs = self._next_observation()
        posAgent = obs[0:3]
        posTarget = obs[3:6]

        if (abs(posAgent[0])<self.max_roi and abs(posAgent[1])<self.max_roi and abs(posAgent[2])<self.max_roi) and self.oob == False:
            resetDone = True
        else:
            resetDone = False
            print("Manual Reset")

        while resetDone == False:
            obs = self.getPositions()
            self.show_renders()

            if keyboard.is_pressed('q'):
                keyboard.wait('q', suppress=True, trigger_on_release=True)
                resetDone = True
                self.oob = False
                self.max_steps_reach = False

            elif keyboard.is_pressed('up'):
                print("Up")
                pwm = "3.89 5.76 5.53 3.66 3.40 1.53 3.60 5.48\n"
                self.con1.sendPWM(pwm)
                keyboard.wait('up', suppress=True, trigger_on_release=True)
                self.con1.sendStop()
                print("stop")

            elif keyboard.is_pressed('down'):
                print("down")
                pwm = "5.53 3.66 3.89 5.76 3.60 5.48 3.40 1.53\n"
                self.con1.sendPWM(pwm)
                keyboard.wait('down', suppress=True, trigger_on_release=True)
                self.con1.sendStop()
                print("stop")

            elif keyboard.is_pressed('right'):
                print("right")
                pwm = "3.66 3.89 5.76 5.53 5.48 3.40 1.53 3.60\n"
                self.con1.sendPWM(pwm)
                keyboard.wait('right', suppress=True, trigger_on_release=True)
                self.con1.sendStop()
                print("stop")

            elif keyboard.is_pressed('left'):
                print("left")
                pwm = "5.76 5.53 3.66 3.89 1.53 3.60 5.48 3.40\n"
                self.con1.sendPWM(pwm)
                keyboard.wait('left', suppress=True, trigger_on_release=True)
                self.con1.sendStop()
                print("stop")

        thres_wksp = 0.4
        if obs[0] > thres_wksp and obs[1] > thres_wksp:
            theta = random.randint(180, 270)
        elif obs[0] < -thres_wksp and obs[1] < -thres_wksp:
            theta = random.randint(0, 90)
        elif obs[0] > thres_wksp:
            theta = random.randint(90, 270)
        elif obs[0] < -thres_wksp:
            theta = random.choice([random.randint(270, 360), random.randint(0, 90)])
        elif obs[1] > thres_wksp:
            theta = random.randint(180, 360)
        elif obs[1] < -thres_wksp:
            theta = random.randint(0, 180)

        else:
            theta = random.randint(0, 360)

        if obs[2] > 0.3:
            phi = random.randint(90, 120)
        elif obs[2] < 0.15:
            phi = random.randint(60, 90)
        else:
            phi = random.randint(60, 120)

        x_tr = self.target_th * np.cos(np.deg2rad(theta)) * np.sin(np.deg2rad(phi)) + obs[0]
        y_tr = self.target_th * np.sin(np.deg2rad(theta)) * np.sin(np.deg2rad(phi)) + obs[1]
        z_tr = self.target_th * np.cos(np.deg2rad(phi)) + obs[2]

        self.posTargetxyz = np.array([x_tr, y_tr, z_tr])

        self.posTargetxyz = np.clip(self.posTargetxyz, -1, 1)

        self.reward = 0

        self.reward=0

        return self._next_observation()

    def render(self, mode='human', close=False):
        self.con1.sendStop()

    def no_current(self):
        self.con1.sendStop()

    def show_renders(self):
        cv2.namedWindow("Cam 1 Feed", cv2.WINDOW_NORMAL)
        cv2.imshow("Cam 1 Feed", self.render_cam1)
        cv2.waitKey(1)

        cv2.namedWindow("Cam 2 Feed", cv2.WINDOW_NORMAL)
        cv2.imshow("Cam 2 Feed", self.render_cam2)
        cv2.waitKey(1)

    def normalize(self, arr, t_min, t_max):
        norm_arr = []
        diff = t_max - t_min
        diff_arr = max(arr) - min(arr)
        for i in arr:
            temp = (((i - min(arr)) * diff) / diff_arr) + t_min
            norm_arr.append(temp)
        return norm_arr

    def reset_mid_episode(self):
        resetDone = False
        while resetDone == False:
            obs = self.getPositions()
            self.show_renders()

            if keyboard.is_pressed('q'):
                keyboard.wait('q', suppress=True, trigger_on_release=True)
                resetDone = True
                self.oob = False

            elif keyboard.is_pressed('up'):
                print("Up")
                pwm = "3.89 5.76 5.53 3.66 3.40 1.53 3.60 5.48\n"
                self.con1.sendPWM(pwm)
                keyboard.wait('up', suppress=True, trigger_on_release=True)
                self.con1.sendStop()
                print("stop")

            elif keyboard.is_pressed('down'):
                print("down")
                pwm = "5.53 3.66 3.89 5.76 3.60 5.48 3.40 1.53\n"
                self.con1.sendPWM(pwm)
                keyboard.wait('down', suppress=True, trigger_on_release=True)
                self.con1.sendStop()
                print("stop")

            elif keyboard.is_pressed('right'):
                print("right")
                pwm = "3.66 3.89 5.76 5.53 5.48 3.40 1.53 3.60\n"
                self.con1.sendPWM(pwm)
                keyboard.wait('right', suppress=True, trigger_on_release=True)
                self.con1.sendStop()
                print("stop")

            elif keyboard.is_pressed('left'):
                print("left")
                pwm = "5.76 5.53 3.66 3.89 1.53 3.60 5.48 3.40\n"
                self.con1.sendPWM(pwm)
                keyboard.wait('left', suppress=True, trigger_on_release=True)
                self.con1.sendStop()
                print("stop")