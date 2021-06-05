"""This file is for testing CCIP

Currently just plots for a test value to check whether it matches an accurate plane drop"""

import math
from matplotlib import pyplot  as plt
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

# Test 1
steps = 2000
Model = CCRP.CCIP(100, 100, 0)

planeheight = [0 for i in range(steps)]
payloadU = [0 for i in range(steps)]

payloadV = [0 for i in range(steps)]

payloadW = [0 for i in range(steps)]
planeEast = [0 for i in range(steps)]
payloady = [0 for i in range(steps)]
payloadx = [0 for i in range(steps)]
payloadz = [0 for i in range(steps)]
planeNorth = [0 for i in range(steps)]
t_data = [i * VPC.dT for i in range(steps)]
reference = Model.createReference()
gains = Controls.controlGains(3.0, 0.04, 0.001, 2.0, 2.0, 5.0, 2.0, -10.0, -0.8, 0.08, 0.03, 2.0, 1.0, -0.5, -0.1)
Model.closed.setControlGains(gains)
for i in range(steps):
    reference = Model.Update(reference)
    payloadU[i] = Model.payload.state.u
    payloadV[i] = Model.payload.state.v
    payloadW[i] = Model.payload.state.w

    planeEast[i] = Model.closed.VAM.vehicle.state.pe
    planeNorth[i] = Model.closed.VAM.vehicle.state.pn
    planeheight[i] = -Model.closed.VAM.vehicle.state.pd

    payloadz[i] = -Model.payload.state.pd
    payloady[i] = Model.payload.state.pe
    payloadx[i] = Model.payload.state.pn
fig, ax = plt.subplots(nrows=3, ncols=3)
ax[0, 0].plot(t_data, planeheight, label="Plane Height")
ax[0, 1].plot(t_data, planeEast, label="Plane East")
ax[0, 2].plot(t_data, planeNorth, label="Plane North")
ax[1, 2].plot(t_data, payloadU, label="payload u")

ax[1, 1].plot(t_data, payloadV, label="payload v")

ax[1, 0].plot(t_data, payloadW, label="payload w")

ax[2, 0].plot(t_data, payloadz, label="payload Z")
ax[2, 1].plot(t_data, payloady, label="payload Y")
ax[2, 2].plot(t_data, payloadx, label="payload X")
ax[0,0].legend()
ax[0,1].legend()
ax[0,2].legend()

ax[1,0].legend()
ax[1,1].legend()
ax[1,2].legend()

ax[2,0].legend()
ax[2,1].legend()
ax[2,2].legend()

plt.show()
