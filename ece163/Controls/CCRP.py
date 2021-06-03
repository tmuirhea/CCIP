import math
import VehicleClosedLoopControl as VCLC
from ece163.Containers import States
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Containers.Controls as ctrl

class PayloadAerodynamicModel:

    def __init__(self, pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0,mass = 10,dT = VPC.dT):
        self.state = States.vehicleState()
        self.state.pn = pn
        self.state.pe = pe
        self.state.pd = pd
        self.state.u = u
        self.state.v = v
        self.state.w = w
        self.mass = mass
        self.dT = dT
    def calculateFuturePos(self,time):
        pn = self.state.pn + self.state.u * time
        pe = self.state.pe + self.state.v * time
        pd = self.state.pd + self.state.w * time
        return pn,pe,pd
    def calculateTOF(self,mass, planArea, cofDrag):
        z = [self.state.pd]
        v = [self.state.w]
        i = 0
        #assuming 1m radius sphere
        while z[i] < 0:
            v.append(v[i] + self.dT * (VPC.g0 - (VPC.rho * planArea* cofDrag * v[i] ** 2)))
            z.append(z[i] + self.dT * v[i])
            i += 1
        return i * self.dT

class CCIP:
    def __init__(self):
        self.closed = VCLC.VehicleClosedLoopControl()
        self.payload = PayloadAerodynamicModel()
    def createReference(self):
        x,y = self.payload.state.pe - self.closed.getVehicleState().pe ,self.payload.state.pn - self.closed.getVehicleState().pn
        course  = math.pi/2 - math.atan2(y,x)
        reference = ctrl.referenceCommands(courseCommand=course)
        return reference

