import math
import VehicleClosedLoopControl as VCLC
from ece163.Containers import States
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Containers.Controls as ctrl


class PayloadAerodynamicModel:
    # default assumes sphere of radius 1m and slow speeds for coefficient of drag
    def __init__(self, pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, dT=VPC.dT, mass=10, planArea=math.pi, cofDrag=0.5):
        self.state = States.vehicleState()
        self.state.pn = pn
        self.state.pe = pe
        self.state.pd = pd
        self.state.u = u
        self.state.v = v
        self.state.w = w
        self.planArea = planArea
        self.cofDrag = cofDrag
        self.mass = mass
        self.dT = dT

    def calculateFuturePos(self, time):
        #very simplistic does not account for wind or drag as is.
        ## should pd be included in here?
        ## I wanted to do futurepos in the TOF while loop but was unsure of converting u,v to pn,pe
        pn = self.state.pn + self.state.u * time
        pe = self.state.pe + self.state.v * time
        pd = self.state.pd + self.state.w * time
        return pn, pe, pd

    def Update(self):
        self.pn = self.state.pn + self.state.u * self.dT
        self.pe = self.state.pe + self.state.v * self.dT
        self.pd = self.state.pd + self.state.w * self.dT
        magnitude = math.hypot(self.u, self.v, self.w)  # needed for calculating drag
        #one timestep of updating speeds which is just drag and gravity in the case of the z direction
        self.u = self.u - self.dT * (VPC.rho * self.planArea * self.cofDrag * magnitude * self.u) / self.mass
        self.v = self.v - self.dT * (VPC.rho * self.planArea * self.cofDrag * magnitude * self.v) / self.mass
        self.w = self.w + self.dT * (VPC.g0 - (VPC.rho * self.planArea * self.cofDrag * magnitude * self.w) / self.mass)

        ##WIND?

class CCIP:
    def __init__(self, mass, targetx, targety, targetz):
        self.closed = VCLC.VehicleClosedLoopControl()
        self.payload = PayloadAerodynamicModel()
        self.dT = VPC.dT
        self.TOF = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.targetx = targetx
        self.targety = targety
        self.targetz = targetz
        self.isreleased = 0

    #for acquiring a new target
    def acquireTarget(self, targetx, targety, targetz):
        self.targetx = targetx
        self.targety = targety
        self.targetz = targetz

    def createReference(self):
        x, y = self.targety - self.closed.getVehicleState().pe, self.targetx - self.closed.getVehicleState().pn
        course = math.pi / 2 - math.atan2(y, x)
        reference = ctrl.referenceCommands(courseCommand=course)
        return reference

    ##added variables here to allow for releasing different payloads
    def releasePayload(self, state, mass=10, area=math.pi, cofDrag=0.5):
        self.payload = PayloadAerodynamicModel(state.pn, state.pe, state.pd, state.u, state.v, state.w, self.dT, mass,
                                               area, cofDrag)

    def calculateTOF(self, payload):
        z = [payload.state.pd]
        v = [payload.state.w]
        i = 0
        while z[i] < 0:
            magnitude = math.hypot(payload.state.u, payload.state.v, v[i])  # needed for calculating drag
            v.append(v[i] + self.dT * (
                        VPC.g0 - (VPC.rho * payload.planArea * payload.cofDrag * magnitude * v[i]) / payload.mass))
            z.append(z[i] + self.dT * v[i])
            i += 1
        return i * self.dT

    def Update(self, RefCommand):
        self.closed.Update(RefCommand)
        self.TOF = self.calculateTOF(self.payload)
        self.x, self.y, self.z = self.payload.calculateFuturePos(self.TOF)
        # if targets line up
        if ((self.x - self.targetx) < .5):  # if x coordinates withing half meter
            if ((self.y - self.targety) < .5):
                if ((self.z - self.targetz) < .5):
                    self.releasePayload(self.closed.VAM.VDM.state)
                    self.isreleased = 1
        if (self.isreleased == 1):  # if payload is released do an Update() amd check for impact
            self.payload.Update()
            ## check for impact?
        refCommand = self.createReference()
        return refCommand
