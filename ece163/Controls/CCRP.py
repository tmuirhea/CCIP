import math
from ece163.Controls import VehicleClosedLoopControl as VCLC
from ece163.Containers import States
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Containers.Controls as ctrl


class PayloadAerodynamicModel:
    # default assumes sphere of radius 1m and slow speeds for coefficient of drag
    def __init__(self, pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0, dT=VPC.dT, mass=10, planArea=math.pi, cofDrag=0.5,released = 0):
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
        self.released = released
    def calculateFuturePos(self, time,state,dot):
        #very simplistic does not account for wind or drag as is.
        ## should pd be included in here?
        ## I wanted to do futurepos in the TOF while loop but was unsure of converting u,v to pn,pe
        pn = state.pn + (dot.u * time)
        pe = state.pe + (dot.v * time)
        return pn, pe
    def reset(self):
        self.state = States.vehicleState()
    def Update(self):
        if self.released == 1:
            self.state.pn = self.state.pn + self.state.u * self.dT
            self.state.pe = self.state.pe + self.state.v * self.dT
            self.state.pd = self.state.pd + self.state.w * self.dT
            if(self.state.pd < 0.0):
                magnitude = math.hypot(self.state.u, self.state.v, self.state.w)  # needed for calculating drag
                #one timestep of updating speeds which is just drag and gravity in the case of the z direction
                self.state.u = self.state.u - self.dT * (VPC.rho * self.planArea * self.cofDrag * magnitude * self.state.u) / (self.mass*2)
                self.state.v = self.state.v - self.dT * (VPC.rho * self.planArea * self.cofDrag * magnitude * self.state.v) / (self.mass*2)
                self.state.w = self.state.w + self.dT * (VPC.g0 - (VPC.rho * self.planArea * self.cofDrag * magnitude * self.state.w) / (self.mass*2))
            else :
                self.state.u = 0.0
                self.state.v = 0.0
                self.state.w = 0.0
                self.state.pd = 0.0
        ##WIND?
        ### Alex: I think wind is calulated
        ### in update forces. I think we just need to call it

class CCIP:
    def __init__(self, targetx = 100.0, targety = 100.0, targetz = 0.0):
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
    def getTarget(self):
        return self.targetx,self.targety,self.targetz
    #reset should set the payload back to nothing, set closed back to
    # intial state,target to intital. 
    def reset(self):
        self.closed.reset()
        self.payload.reset()
        self.x = 0
        self.y = 0
        self.targetx = 10.0
        self.targety = 10.0
        self.targetz = 0.0
        
    #added some state setters and getters
    def setVehicleState(self,state):
        self.closed.VAM.vehicle.state = state
    def getVehicleState(self):
        return self.closed.VAM.vehicle.state
    
    def createReference(self):
        x, y = self.targety - self.closed.getVehicleState().pe, self.targetx - self.closed.getVehicleState().pn
        course = math.pi / 2 - math.atan2(y, x)
        reference = ctrl.referenceCommands(courseCommand=course,altitudeCommand = 1000)
        return reference

    ##added variables here to allow for releasing different payloads
    def releasePayload(self, mass=10, area=math.pi, cofDrag=0.5):
        state = self.closed.getVehicleState()
        dot = self.closed.VAM.vehicle.dot
        self.payload = PayloadAerodynamicModel(state.pn, state.pe, state.pd, dot.pn, dot.pe, dot.pd, self.dT, mass,
                                               area, cofDrag, 1)
    def calculateTOF(self, state,planArea,cofDrag,mass):
        z = [state.pd]
        
        v = [state.w]
        i = 0
        while z[i] < 0:
            magnitude = math.hypot(state.u, state.v, v[i])  # needed for calculating drag
            v.append(v[i] + self.dT * (
                        VPC.g0 - (VPC.rho * planArea * cofDrag * magnitude * v[i]) / (mass*2)))
            z.append(z[i] + self.dT * v[i])
            i += 1
        return i * self.dT
    #IsImapcted returns 1/0 if payload intersects target
    def isImpacted(self):
        if abs(self.payload.state.pn - self.targetx) < 10:
            if abs(self.payload.state.pe - self.targety) < 10:
                if abs(self.payload.state.pd - self.targetz) < 1:
                    print(self.payload.state.pn)
                    print(self.payload.state.pe)
                    print(self.payload.state.pd)
                    return 1
    def Update(self, RefCommand):
        self.closed.Update(RefCommand)
        self.TOF = self.calculateTOF(self.closed.VAM.vehicle.state,math.pi,.5,25)
        self.x, self.y = self.payload.calculateFuturePos(self.TOF, self.closed.VAM.vehicle.state, self.closed.VAM.vehicle.dot)
        # if targets line up

        if (abs(self.x - self.targetx) < 5):  # if x coordinates withing half meter
            if (abs(self.y - self.targety) < 1):
                if self.payload.released == 0:
                    self.releasePayload()
                    print("released")
        if (self.payload.released == 1):  # if payload is released do an Update() amd check for impact
            self.payload.Update()
            
            Impact = self.isImpacted()
            #if impact "delete" payload and print
            if Impact == 1:
                print("IMPACT!!!!\n")
        RefCommand = self.createReference()
        return RefCommand
