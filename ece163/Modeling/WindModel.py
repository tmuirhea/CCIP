"""WindModel Class
Models winds with Dryden estimates

Code Author: Kevin Jesubalan (kjesubal@ucsc.edu)
"""
import math
import random
from ..Containers import States
from ..Utilities import MatrixMath as mm
from ..Constants import VehiclePhysicalConstants as VPC


class WindModel:

    def __init__(self, dT=VPC.dT, Va=VPC.InitialSpeed, drydenParamters=VPC.DrydenNoWind):
        # intitalize state variables and wind state
        self.xu = 0
        self.xv = [[0], [0]]
        self.xw = [[0], [0]]
        self.windstate = States.windState()

        # next got to intialize discrete state models and have two cases depending on whether NoWind is passed

        # if any other wind model passed we can use CDTF()

        self.CreateDrydenTransferFns(dT, Va, drydenParamters)

    # resets everything except for Drydens which have to be changed with CDTF()
    def reset(self):
        self.xu = 0
        self.xv = [[0], [0]]
        self.xw = [[0], [0]]
        self.windstate = States.windState()

    # just sets wind state equal to wind state passed
    def setWind(self, windState):
        self.windstate = windState

    # return the models wind state attribute
    def getWind(self):
        return self.windstate

    # changes Windmodel by adding steady winds along with drydenParameters to make gusts
    def setWindModelParameters(self, Wn=0.0, We=0.0, Wd=0.0, drydenParamters=VPC.DrydenNoWind):
        self.windstate.Wn = Wn
        self.windstate.We = We
        self.windstate.Wd = Wd
        self.CreateDrydenTransferFns(VPC.dT, VPC.InitialSpeed, drydenParamters)

    def CreateDrydenTransferFns(self, dT, Va, drydenParamters):
        # snagged this off piazza to account for 0's in the nowind parameter
        if drydenParamters == VPC.DrydenNoWind:
            self.Phi_u = [[1.0]]
            self.Gamma_u = [[0.0]]
            self.H_u = [[1.0]]

            self.Phi_v = [[1.0, 0.0], [0.0, 1.0]]
            self.Gamma_v = [[0.0], [0.0]]
            self.H_v = [[1.0, 1.0]]

            self.Phi_w = [[1.0, 0.0], [0.0, 1.0]]
            self.Gamma_w = [[0.0], [0.0]]
            self.H_w = [[1.0, 1.0]]

        else:

            # solving for state variables for u (all scalar)
            self.Phi_u = [[math.exp(-Va * dT / drydenParamters.Lu)]]
            self.Gamma_u = [[(drydenParamters.Lu / Va) * (1 - math.exp(-Va * dT / drydenParamters.Lu))]]
            self.H_u = [[drydenParamters.sigmau * math.sqrt(2 * Va / (math.pi * drydenParamters.Lu))]]

            # solving for state matrices of v
            self.Phi_v = mm.scalarMultiply(math.exp(-Va * dT / drydenParamters.Lv),
                                           [[1 - Va * dT / drydenParamters.Lv, -(Va / drydenParamters.Lv) ** 2 * dT],
                                            [dT, 1 + Va * dT / drydenParamters.Lv]])
            self.Gamma_v = mm.scalarMultiply(math.exp(-Va * dT / drydenParamters.Lv),
                                             [[dT], [(drydenParamters.Lv / Va) ** 2 * (math.exp(
                                                 Va * dT / drydenParamters.Lv) - 1) - drydenParamters.Lv * dT / Va]])
            self.H_v = mm.scalarMultiply(drydenParamters.sigmav * math.sqrt(3 * Va / (math.pi * drydenParamters.Lv)),
                                         [[1, Va / (math.sqrt(3) * drydenParamters.Lv)]])

            # Solving for state matrices of w
            self.Phi_w = mm.scalarMultiply(math.exp(-Va * dT / drydenParamters.Lw),
                                           [[1 - Va * dT / drydenParamters.Lw, -(Va / drydenParamters.Lw) ** 2 * dT],
                                            [dT, 1 + Va * dT / drydenParamters.Lw]])
            self.Gamma_w = mm.scalarMultiply(math.exp(-Va * dT / drydenParamters.Lw),
                                             [[dT], [(drydenParamters.Lw / Va) ** 2 * (math.exp(
                                                 Va * dT / drydenParamters.Lw) - 1) - drydenParamters.Lw * dT / Va]])
            self.H_w = mm.scalarMultiply(drydenParamters.sigmaw * math.sqrt(3 * Va / (math.pi * drydenParamters.Lw)),
                                         [[1, Va / (math.sqrt(3) * drydenParamters.Lw)]])

    def getDrydenTransferFns(self):
        return self.Phi_u, self.Gamma_u, self.H_u, self.Phi_v, self.Gamma_v, self.H_v, self.Phi_w, self.Gamma_w, self.H_w

    def Update(self, uu=None, uv=None, uw=None):

        if uu is None:
            uu = random.gauss(0, 1)

        if uv is None:
            uv = random.gauss(0, 1)

        if uw is None:
            uw = random.gauss(0, 1)

        xu_1 = self.Phi_u[0][0] * self.xu + self.Gamma_u[0][0] * uu
        self.windstate.Wu = self.H_u[0][0] * xu_1
        self.xu = xu_1

        xv_1 = mm.add(mm.multiply(self.Phi_v, self.xv) , mm.scalarMultiply(uv, self.Gamma_v))
        self.windstate.Wv = mm.multiply(self.H_v, xv_1)[0][0]
        self.xv = xv_1

        xw_1 = mm.add(mm.multiply(self.Phi_w, self.xw) , mm.scalarMultiply(uw, self.Gamma_w))
        self.windstate.Ww = mm.multiply(self.H_w, xw_1)[0][0]
        self.xw = xw_1
