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
steps = 1000
Model = CCRP.PayloadAerodynamicModel()
Model.state.pd = 100
gyroxTrue = [0 for i in range(steps)]
t_data = [i*VPC.dT for i in  range(steps)]
for i in range (steps):
        Model.Update()
        gyroxTrue[i] = Model.state.pd
fig , ax = plt.subplots(nrows=3, ncols=3)
ax[0, 0].plot(t_data , gyroxTrue, label = "gyrox True")

ax[0, 0].legend ()

plt.show()
