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
    def Update(self)
        
class CCIP:
    def __init__(self,mass,targetx,targety,targetz):
        self.closed = VCLC.VehicleClosedLoopControl()
        self.payload = PayloadAerodynamicModel()
        self.dT = VPC.dT
        self.TOF = 0
        self.mass = mass;
        self.area = 1;
        self.cofDrag = .5
        self.x = 0
        self.y = 0
        self.z = 0
        self.targetx = targetx
        self.targety = targety
        self.targetz = targetz
        self.isreleased = 0
    def createReference(self):
        x,y = self.targety - self.closed.getVehicleState().pe ,self.targetx - self.closed.getVehicleState().pn
        course  = math.pi/2 - math.atan2(y,x)
        reference = ctrl.referenceCommands(courseCommand=course)
        return reference
    
    def releasePayload(self,state,mass,dT):
        self.payload = PayloadAerodynamicModel(state.pn,state.pe,state.pd,state.u,state.v,state.w,mass,dT)
        
    def calculateTOF(self,state, mass, planArea, cofDrag, dT):
        z = [state.pd]
        v = [state.w]
        i = 0
        #assuming 1m radius sphere
        while z[i] < 0:
            v.append(v[i] + self.dT * (VPC.g0 - (VPC.rho * planArea* cofDrag * v[i] ** 2)))
            z.append(z[i] + self.dT * v[i])
            i += 1
        return i * self.dT

    def Update(self,RefCommand):
        self.VCLC.Update(RefCommand)
        self.TOF = calculateTOF(self.closed.VAM.VDM.state, self.mass,self.area,self.cofDrag,self.dT)
        self.x,self.y,self.z = self.payload.calculateFuturePos(self.TOF)
        #if targets line up
        if ((self.x- self.targetx) < .5 ): #if x coordinates withing half meter
            if ((self.y- self.targety) < .5 ):
                if ((self.z- self.targetz) < .5 ):
                    releasePayload(self.closed.VAM.VDM.state,self.mass,self.dT)
                    self.isreleased = 1
        if (self.isreleased == 1): #if payload is released check for impact
            #check for impact
        refCommand = self.createReference()
        return RefCommand
        
