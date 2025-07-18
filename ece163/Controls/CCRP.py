import math
from ece163.Controls import VehicleClosedLoopControl as VCLC
from ece163.Containers import States
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Containers.Controls as ctrl


class PayloadAerodynamicModel:
    # default assumes sphere of radius 1m and slow speeds for coefficient of drag
    def __init__(self, pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, dT=VPC.dT, planArea=math.pi, cofDrag=0.5, mass=10):
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
        self.released = False

    def calculateFuturePos(self, time, state, dot):
        # very simplistic does not account for wind or drag as is.
        ## should pd be included in here?
        ## I wanted to do futurepos in the TOF while loop but was unsure of converting u,v to pn,pe
        pn = state.pn + (dot.u * time)
        pe = state.pe + (dot.v * time)
        return pn, pe

    def reset(self):
        self.state = States.vehicleState()

    def Update(self):
        if self.released == True:
            self.state.pn = self.state.pn + self.state.u * self.dT
            self.state.pe = self.state.pe + self.state.v * self.dT
            self.state.pd = self.state.pd + self.state.w * self.dT
            if (self.state.pd < 0.0):

                magnitude = math.hypot(self.state.u, self.state.v, self.state.w)  # needed for calculating drag

                # one timestep of updating speeds which is just drag and gravity in the case of the z direction
                self.state.u = self.state.u - self.dT * (
                        VPC.rho * self.planArea * self.cofDrag * magnitude * self.state.u) / (self.mass * 2)
                self.state.v = self.state.v - self.dT * (
                        VPC.rho * self.planArea * self.cofDrag * magnitude * self.state.v) / (self.mass * 2)
                self.state.w = self.state.w + self.dT * (
                        VPC.g0 - (VPC.rho * self.planArea * self.cofDrag * magnitude * self.state.w) / (
                        self.mass * 2))
            else:
                self.state.u = 0.0
                self.state.v = 0.0
                self.state.w = 0.0
                self.state.pd = 0.0
        ##WIND?
        ### Alex: I think wind is calulated
        ### in update forces. I think we just need to call it


class CCIP:
    def __init__(self, targetx=100.0, targety=100.0):
        self.closed = VCLC.VehicleClosedLoopControl()
        self.payload = PayloadAerodynamicModel()
        self.dT = VPC.dT
        self.TOF = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.targetx = targetx
        self.targety = targety
        self.tooclose = False
        self.hitground = False
        self.initialTime = 0
        self.timeCount = 0

    # for acquiring a new target
    def acquireTarget(self, targetx, targety):
        self.targetx = targetx
        self.targety = targety

    def getTarget(self):
        return self.targetx, self.targety, self.targetz

    # reset should set the payload back to nothing, set closed back to
    # intial state,target to intital. 
    def reset(self):
        self.closed.reset()
        self.payload.reset()
        self.x = 0
        self.y = 0
        self.targetx = 10.0
        self.targety = 10.0
        self.targetz = 0.0
        self.tooclose = False
        self.hitground = False
        self.timeCount = 0

    # added some state setters and getters
    def setVehicleState(self, state):
        self.closed.VAM.vehicle.state = state

    def getVehicleState(self):
        return self.closed.VAM.vehicle.state

    def createReference(self):
        x, y = self.targety - self.closed.getVehicleState().pe, self.targetx - self.closed.getVehicleState().pn
        course = math.pi / 2 - math.atan2(y, x)
        if self.payload.released == False:
            reference = ctrl.referenceCommands(courseCommand=course)
        else:
            reference = ctrl.referenceCommands(courseCommand=self.closed.getVehicleState().chi)
        return reference

    ##added variables here to allow for releasing different payloads
    def releasePayload(self, area=math.pi, cofDrag=0.5, mass=10):
        state = self.closed.getVehicleState()
        dot = self.closed.VAM.vehicle.dot
        self.payload = PayloadAerodynamicModel(state.pn, state.pe, state.pd, dot.pn, dot.pe, dot.pd, self.dT,
                                               area, cofDrag, mass)
        self.payload.released = True

    def calculateTOF(self, state, dot, planArea, cofDrag, mass):
        x = [state.pn]
        y = [state.pe]
        z = [state.pd]
        u = [dot.pn]
        v = [dot.pe]
        w = [dot.pd]

        i = 0
        while z[i] < 0:
            magnitude = math.hypot(u[i], v[i], w[i])  # needed for calculating drag
            u.append(u[i] - self.dT * (VPC.rho * planArea * cofDrag * magnitude * u[i]) / (mass * 2))
            v.append(v[i] - self.dT * (VPC.rho * planArea * cofDrag * magnitude * v[i]) / (mass * 2))
            w.append(w[i] + self.dT * (
                    VPC.g0 - (VPC.rho * planArea * cofDrag * magnitude * w[i]) / (mass * 2)))

            x.append(x[i] + self.dT * u[i])
            y.append(y[i] + self.dT * v[i])
            z.append(z[i] + self.dT * w[i])
            i += 1
        return x[i], y[i]

    # IsImapcted returns 1/0 if payload intersects target
    def isImpacted(self):
        if self.payload.released and not self.hitground and -self.payload.state.pd < 1:
            print("Impact Coordinates")
            print("pn:" , self.payload.state.pn, "pe:", self.payload.state.pe, self.timeCount * self.dT , "secs")
            self.hitground = True

    def Update(self, RefCommand):
        self.closed.Update(RefCommand)
        self.x, self.y = self.calculateTOF(self.closed.VAM.vehicle.state, self.closed.VAM.vehicle.dot, math.pi, .5, 25)

        # print(self.x, self.y, self.payload.released,self.tooclose)
        # if targets line up
        if math.hypot(self.x - self.targetx, self.y - self.targety) < 3:
            if not self.payload.released:
                self.releasePayload(math.pi, .5, 25)
                print("released at",  self.timeCount * self.dT,"secs")
        # this is to check whether the plane needs to make another pass
        distanceError = math.hypot(self.x - self.targetx, self.y - self.targety)
        planeDistanceFromTarget = math.hypot(self.targetx - self.getVehicleState().pn,
                                             self.targety - self.getVehicleState().pe)
        # print(distanceError, 2 * planeDistanceFromTarget)
        if distanceError > 2 * planeDistanceFromTarget:
            self.tooclose = True
        else:
            self.tooclose = False

        if self.payload.released:  # if payload is released do an Update() amd check for impact
            self.payload.Update()

        self.isImpacted()

        if not self.tooclose:  # this check is for holding course for a bit to so the plane can make another run
            RefCommand = self.createReference()

        self.timeCount += 1
        return RefCommand
