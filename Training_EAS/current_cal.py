import numpy as np
from timeit import default_timer as timer
import time

class current_cal_class:
    def __init__(self):
        self.B_Vec = np.array([[-0.3784, -0.6537, 0.3784, 0.6537, -0.4818, -0.1650, 0.4818, 0.1650],
                          [-0.6537, 0.3784, 0.6537, -0.3784, -0.1650, 0.4818, 0.1650, -0.4818],
                          [0.0457, 0.0457, 0.0457, 0.0457, 0.6525, 0.6525, 0.6525, 0.6525]])
        self.B_Vec = self.B_Vec * 0.001

        self.Grad_X = np.array([[-0.0195, -0.0057, -0.0195, -0.0057, 0.0202, -0.0344, 0.0202, -0.0344],
                           [-0.0189, 0.0160, -0.0189, 0.0160, 0	,    0  ,        0,	       0],
                           [0.0118,    0.0128,   -0.0118,    -0.0128,    0.0369,    0,	       -0.0369,     0]])

        self.Grad_Y = np.array([[-0.0160,    0.0189,   -0.0160,    0.0189,    0,	    0 ,        0,	        0],
                           [-0.0057,   -0.0195,    -0.0057,   -0.0195,   -0.0344,   0.0202,   -0.0344,    0.0202],
                           [0.0128,   -0.0118,   -0.0128,    0.0118,      0,        -0.0369,   0,         0.0369]])

        self.Grad_Z = np.array([[0.0078,    0.0069,   -0.0078,   -0.0069,    0.0344,    0,	     -0.0344,    0],
                           [0.0069,   -0.0078,   -0.0069,    0.0078,    0,	       -0.0344,    0,         0.0344],
                           [0.0136,    0.0136,    0.0136,    0.0136,   -0.0183,   -0.0183,   -0.0183,   -0.0183]])

    def get_coil_currents(self, B, B_unit, Grad):

        MagneticFieldAmplitude_Value = B

        Grad_XValue = Grad[0];
        GradY_Value = Grad[1];
        GradZ_Value = Grad[2];

        if MagneticFieldAmplitude_Value > 20:
            MagneticFieldAmplitude_Value = 20

        if Grad_XValue > 150:
            Grad_XValue = 150

        if GradY_Value > 150:
            GradY_Value = 150

        if GradZ_Value > 150:
            GradZ_Value = 150

        Grequired = np.array([[Grad_XValue, GradY_Value, GradZ_Value]]) / 1000
        B_mag = MagneticFieldAmplitude_Value / 1000
        B = B_mag * B_unit
        Bunit = 100 * np.transpose(B)
        Brequired = B_mag * Bunit

        ActuationMatrix = self.B_Vec
        ActuationMatrix = np.append(ActuationMatrix, np.dot(Bunit, self.Grad_X), axis=0)
        ActuationMatrix = np.append(ActuationMatrix, np.dot(Bunit, self.Grad_Y), axis=0)
        ActuationMatrix = np.append(ActuationMatrix, np.dot(Bunit, self.Grad_Z), axis=0)

        Inverse_AM = np.linalg.pinv(ActuationMatrix)
        temp = np.transpose(Brequired)
        temp = np.append(temp, np.transpose(Grequired))  # [Brequired';Grequired']
        I = np.dot(Inverse_AM, temp)
        return(I)

current_cal = current_cal_class()

start_time = timer()
current_cal.get_coil_currents(10, np.array([[0],[0],[1]]), np.array([0, 100, 0]))
print("Time taken = " + str(timer() - start_time))

start_time = timer()
current_cal.get_coil_currents(10, np.array([[0],[0],[1]]), np.array([600, 600, 0]))
print("Time taken = " + str(timer() - start_time))

start_time = time.time_ns()
current_cal.get_coil_currents(10, np.array([[0],[0],[1]]), np.array([600, 600, 0]))
current_cal.get_coil_currents(10, np.array([[0],[0],[1]]), np.array([600, 600, 0]))
current_cal.get_coil_currents(10, np.array([[0],[0],[1]]), np.array([600, 600, 0]))
current_cal.get_coil_currents(10, np.array([[0],[0],[1]]), np.array([600, 600, 0]))
time.sleep(1)
end = time.time_ns() - start_time
print(start_time)
print(time.time_ns())
print("Time taken = " + str(end))

start_time = timer()
current_cal.get_coil_currents(10, np.array([[0],[0],[1]]), np.array([600, 600, 0]))
print("Time taken = " + str(timer() - start_time))

#print(currents)



"""
B_Vec = np.array([[-0.3784,   -0.6537,    0.3784 ,   0.6537 ,  -0.4818 ,  -0.1650  ,  0.4818  ,  0.1650],
                  [-0.6537,    0.3784 ,   0.6537 ,  -0.3784 ,  -0.1650  ,  0.4818  ,  0.1650  , -0.4818],
                  [0.0457 ,   0.0457 ,   0.0457 ,   0.0457  ,  0.6525   , 0.6525   , 0.6525   , 0.6525]])
B_Vec = B_Vec*0.001

Grad_X = np.array([[-0.0195,    -0.0057,   -0.0195,   -0.0057,    0.0202,    -0.0344,    0.0202,    -0.0344],
                   [-0.0189,    0.0160,   -0.0189,     0.0160 ,   0	 ,    0  ,        0,	       0],
                   [0.0118,    0.0128,   -0.0118,    -0.0128,    0.0369,    0,	       -0.0369,     0]])

Grad_Y = np.array([[-0.0160,    0.0189,   -0.0160,    0.0189,    0,	    0 ,        0,	        0],
                   [-0.0057,   -0.0195,    -0.0057,   -0.0195,   -0.0344,   0.0202,   -0.0344,    0.0202],
                   [0.0128,   -0.0118,   -0.0128,    0.0118,      0,        -0.0369,   0,         0.0369]])


Grad_Z = np.array([[0.0078,    0.0069,   -0.0078,   -0.0069,    0.0344,    0,	     -0.0344,    0],
                   [0.0069,   -0.0078,   -0.0069,    0.0078,    0,	       -0.0344,    0,         0.0344],
                   [0.0136,    0.0136,    0.0136,    0.0136,   -0.0183,   -0.0183,   -0.0183,   -0.0183]])

MagneticFieldAmplitude_Value=10

Grad_XValue=0;
GradY_Value=100;
GradZ_Value=0;

if MagneticFieldAmplitude_Value>20 :
    MagneticFieldAmplitude_Value=20

if Grad_XValue>150:
    Grad_XValue=150

if GradY_Value>150:
    GradY_Value=150

if GradZ_Value>150:
    GradZ_Value=150


Grequired=np.array([[Grad_XValue, GradY_Value, GradZ_Value]])/1000
B_mag=MagneticFieldAmplitude_Value/1000
B = B_mag * np.array([[0],[0],[1]])
Bunit = 100 * np.transpose(B)
Brequired = B_mag * Bunit

ActuationMatrix = B_Vec
ActuationMatrix = np.append(ActuationMatrix, np.dot(Bunit,Grad_X), axis = 0)
ActuationMatrix = np.append(ActuationMatrix, np.dot(Bunit,Grad_Y), axis = 0)
ActuationMatrix = np.append(ActuationMatrix, np.dot(Bunit,Grad_Z), axis = 0)

Inverse_AM = np.linalg.pinv(ActuationMatrix)
temp = np.transpose(Brequired)
temp = np.append(temp, np.transpose(Grequired)) #[Brequired';Grequired']
I = np.dot(Inverse_AM, temp)
print(I)
print(None)
"""
"""
B_mag=MagneticFieldAmplitude_Value/1000;
B=B_mag*[0;0;1];
Bunit=100*B';
Brequired = B_mag* Bunit;
ActuationMatrix = [B_Vec;Bunit*Grad_X;Bunit*Grad_Y;Bunit*Grad_Z];
Inverse_AM = pinv(ActuationMatrix);
I = (Inverse_AM * [Brequired';Grequired'])'

print(a.shape)

print(a)"""