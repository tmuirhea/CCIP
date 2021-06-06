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

# Test for CCIP
# Next 3 lines are variables you can change for testing purposes
steps = 5000  # number of steps you want to iterate simulation over (time = steps * 0.01)
Model = CCRP.CCIP(100, 100)  # initialize with position of target you want plane to hit (target-x,target-y) z assumed 0
Model.closed.VAM.vehicle.state.pd = -100  # height you want plane to start at (plane will try to return to 100)

# initializations of lists for storing data
planeheight = [0 for i in range(steps)]
payloadU = [0 for i in range(steps)]
payloadV = [0 for i in range(steps)]
payloadW = [0 for i in range(steps)]
planeEast = [0 for i in range(steps)]
payloady = [0 for i in range(steps)]
payloadx = [0 for i in range(steps)]
payloadz = [0 for i in range(steps)]
testpayloadz = [0 for i in range(steps)]
testpayloadchi = [0 for i in range(steps)]
planeNorth = [0 for i in range(steps)]

# the time steps we'll be using
t_data = [i * VPC.dT for i in range(steps)]
# initializations for a reference variable for plane to use
reference = Model.createReference()
# creating some gains and setting our plane to use those gains
gains = Controls.controlGains(3.0, 0.04, 0.001, 2.0, 2.0, 5.0, 2.0, -10.0, -0.8, 0.08, 0.03, 2.0, 1.0, -0.5, -0.1)
Model.closed.setControlGains(gains)
# interating for # of steps
for i in range(steps):
    # plane will fly using Model.update and eventually release a payload setting payload.released to 1
    reference = Model.Update(reference)
    planeEast[i] = Model.closed.VAM.vehicle.state.pe
    planeNorth[i] = Model.closed.VAM.vehicle.state.pn
    planeheight[i] = -Model.closed.VAM.vehicle.state.pd
    # uses plane states until the payload is released (since it is attached)
    if Model.payload.released == 0:
        payloadU[i] = Model.closed.VAM.vehicle.dot.pn
        payloadV[i] = Model.closed.VAM.vehicle.dot.pe
        payloadW[i] = Model.closed.VAM.vehicle.dot.pd
        payloadz[i] = -Model.closed.VAM.vehicle.state.pd
        payloady[i] = Model.closed.VAM.vehicle.state.pe
        payloadx[i] = Model.closed.VAM.vehicle.state.pn

    # checking whether payload has been released
    if Model.payload.released == 1:
        # once the payload is released will start tracking the payload variables
        payloadU[i] = Model.payload.state.u
        payloadV[i] = Model.payload.state.v
        payloadW[i] = Model.payload.state.w
        payloadz[i] = -Model.payload.state.pd
        payloady[i] = Model.payload.state.pe
        payloadx[i] = Model.payload.state.pn

fig, ax = plt.subplots(nrows=3, ncols=3)
ax[0, 0].plot(t_data, planeNorth, 'r', label="Plane North")
ax[0, 1].plot(t_data, planeEast, 'g', label="Plane East")
ax[0, 2].plot(t_data, planeheight, 'b', label="Plane Height")

ax[1, 0].plot(t_data, payloadU, 'r', label=r'payload $\dot{pn}$')
ax[1, 1].plot(t_data, payloadV, 'g', label=r'payload $\dot{pe}$')
ax[1, 2].plot(t_data, payloadW, 'b', label=r'payload $\dot{pd}$')

ax[2, 0].plot(t_data, payloadx, 'r', label="payload pn")
ax[2, 1].plot(t_data, payloady, 'g', label="payload pe")
ax[2, 2].plot(t_data, payloadz, 'b', label="payload height")

ax[0, 0].legend()
ax[0, 0].set_xlabel("time(s)")
ax[0, 0].set_ylabel("position (m)")

ax[0, 1].legend()
ax[0, 1].set_xlabel("time(s)")
ax[0, 1].set_ylabel("position (m)")
ax[0, 1].set_title("Plane Position")

ax[0, 2].legend()
ax[0, 2].set_xlabel("time(s)")
ax[0, 2].set_ylabel("position (m)")


ax[1, 0].legend()
ax[1, 0].set_xlabel("time(s)")
ax[1, 0].set_ylabel("position (m)")

ax[1, 1].legend()
ax[1, 1].set_xlabel("time(s)")
ax[1, 1].set_ylabel("position (m)")
ax[1, 1].set_title("Payload Position")

ax[1, 2].legend()
ax[1, 2].set_xlabel("time(s)")
ax[1, 2].set_ylabel("position (m)")

ax[2, 0].legend()
ax[2, 0].set_xlabel("time(s)")
ax[2, 0].set_ylabel("velocity (m/s)")

ax[2, 1].legend()
ax[2, 1].set_xlabel("time(s)")
ax[2, 1].set_ylabel("velocity (m/s)")
ax[2, 1].set_title("Payload Velocity")

ax[2, 2].legend()
ax[2, 2].set_xlabel("time(s)")
ax[2, 2].set_ylabel("velocity (m/s)")
plt.subplots_adjust(hspace=1)

plt.show()
