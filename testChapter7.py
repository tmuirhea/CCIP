"""This file is a test harness for the module ece163.Utilities.Rotations,
and for the method ece163.Modeling.VehicleGeometry.getNewPoints(). 

 It is meant to be run from the root directory of the repo with:

python testChapter2.py

at which point it will execute various tests on the Rotations module"""

#%% Initialization of test harness and helpers:
#Written by: Alexander Schulte (aschulte@ucsc.edu)
import math
from  matplotlib  import  pyplot  as plt
from ece163.Containers import States
from ece163.Sensors import SensorsModel
from ece163.Containers import Linearized
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Controls import VehicleTrim
from ece163.Controls import VehicleClosedLoopControl as VCLC
from ece163.Controls import VehicleControlGains as vcc
from ece163.Controls import VehiclePerturbationModels
from ece163.Constants import VehiclePhysicalConstants as VPC
import ece163.Utilities.MatrixMath as mm
from ece163.Modeling import WindModel as WM
from ece163.Modeling import VehicleDynamicsModel as Dynamic
import ece163.Utilities.Rotations as Rotations
from ece163.Modeling import VehicleAerodynamicsModel as aero
import ece163.Modeling.VehicleGeometry as VG
from ece163.Controls import CCRP 
import numpy as np

#Test 1
steps = 2000
Model = CCRP.CCIP(100,100,0)
#Model.closed.VAM.vehicle.state.pd = 100

gyroxTrue = [0 for i in range(steps)]
Velocity = [0 for i in range(steps)]
Velocityx = [0 for i in range(steps)]
payloady = [0 for i in range(steps)]
payloadx = [0 for i in range(steps)]
payloadz = [0 for i in range(steps)]
Velocityy = [0 for i in range(steps)]
t_data = [i*VPC.dT for i in  range(steps)]
reference = Model.createReference()
gains = Controls.controlGains(3.0, 0.04,0.001, 2.0, 2.0, 5.0, 2.0,  -10.0, -0.8, 0.08, 0.03, 2.0,  1.0, -0.5, -0.1)
Model.closed.setControlGains(gains)
for i in range (steps):
        reference = Model.Update(reference)
        Velocity[i] = Model.closed.VAM.vehicle.state.u
        Velocityx[i] = Model.closed.VAM.vehicle.state.pe
        Velocityy[i] = Model.closed.VAM.vehicle.state.pn
        gyroxTrue[i] = -Model.closed.VAM.vehicle.state.pd
                
        payloadz[i] = -Model.payload.state.pd
        payloady[i] = Model.payload.state.pe
        payloadx[i] = Model.payload.state.pn
fig , ax = plt.subplots(nrows=3, ncols=3)
ax[0, 0].plot(t_data , gyroxTrue, label = "Z")
ax[0, 1].plot(t_data , Velocityx, label = "Y")
ax[0, 2].plot(t_data , Velocityy, label = "X")
ax[1, 0].plot(t_data , Velocity, label = "Velocityz")

ax[2, 0].plot(t_data , payloadz, label = "payload Z")
ax[2, 1].plot(t_data , payloadx, label = "payload Y")
ax[2, 2].plot(t_data , payloady, label = "payload X")
ax[0, 0].legend()
ax[0, 1].legend ()
ax[0, 2].legend ()
ax[1, 0].legend ()

ax[2, 0].legend()
ax[2, 1].legend()
ax[2, 2].legend ()

plt.show()
