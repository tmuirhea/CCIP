import math
from ..Containers import States
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC


class VehicleDynamicsModel:

    def __init__(self, dT=VPC.dT):
        self.state = States.vehicleState()
        self.dot = States.vehicleState()
        self.dT = dT

    # All my setters and getters are below
    def setVehicleState(self, state):
        self.state = state

    def getVehicleState(self):
        return self.state

    def setVehicleDerivative(self, dot):
        self.dot = dot

    def getVehicleDerivative(self):
        return self.dot

    # resets state and dot to initial values
    def reset(self):
        self.state = States.vehicleState()
        self.dot = States.vehicleState()

    # updates state and dot using Rexp and Derivative
    def Update(self, forcesMoments):
        self.setVehicleDerivative(self.derivative(self.getVehicleState(), forcesMoments))
        self.setVehicleState(self.IntegrateState(self.dT, self.getVehicleState(), self.getVehicleDerivative()))

    def Rexp(self, dT, state, dot):
        eye = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # identity matrix to be used for calculation
        fwdeuler = self.ForwardEuler(dT / 2, state, dot)  # forward euler so I can get p,q,r values
        wvec = [[fwdeuler.p], [fwdeuler.q], [fwdeuler.r]]  # putting those p,q,r values into a vector
        skew = mm.skew(wvec[0][0], wvec[1][0], wvec[2][0])  # getting the skew matrix of w
        wnorm = math.sqrt(wvec[0][0] ** 2 + wvec[1][0] ** 2 + wvec[2][0] ** 2)  # getting the norm of w
        # a ton of math follows taken from AttitudeCheatSheet
        if wnorm < 0.001:
            sinc = dT - (dT ** 3 * wnorm ** 2) / 6 + (dT ** 5 * wnorm ** 4) / 120
            cosc = dT ** 2 / 2 - (dT ** 4 * wnorm ** 2) / 24 + (dT ** 6 * wnorm ** 4) / 720
            expm = mm.add(mm.subtract(eye, mm.scalarMultiply(sinc, skew)),
                          mm.scalarMultiply(cosc, mm.multiply(skew, skew)))
        else:
            expm = mm.add(mm.subtract(eye, mm.scalarMultiply(math.sin(wnorm * dT) / wnorm, skew)),
                          mm.scalarMultiply((1 - math.cos(wnorm * dT)) / wnorm ** 2, mm.multiply(skew, skew)))
        return expm

    def derivative(self, state, forcesMoments):
        dot = States.vehicleState()  # variable that's going to be returned

        speedvec = [[state.u], [state.v], [state.w]]  # current speed in body
        posdot = mm.multiply(mm.transpose(state.R), speedvec)  # getting the ned derivatives
        # setting ned derivatives (inertial velocity)
        dot.pn = posdot[0][0]
        dot.pe = posdot[1][0]
        dot.pd = posdot[2][0]
        # getting body frame acceleration
        dot.u = state.r * state.v - state.q * state.w + forcesMoments.Fx / VPC.mass
        dot.v = state.p * state.w - state.r * state.u + forcesMoments.Fy / VPC.mass
        dot.w = state.q * state.u - state.p * state.v + forcesMoments.Fz / VPC.mass
        # euler angle derivatives
        eulerrotation = [
            [1, math.sin(state.roll) * math.tan(state.pitch), math.cos(state.roll) * math.tan(state.pitch)],
            [0, math.cos(state.roll), -math.sin(state.roll)],
            [0, math.sin(state.roll) / math.cos(state.pitch),
             math.cos(state.roll) / math.cos(state.pitch)]]
        rotvec = [[state.p], [state.q], [state.r]]
        eulerdot = mm.multiply(eulerrotation, rotvec)
        dot.roll = eulerdot[0][0]
        dot.pitch = eulerdot[1][0]
        dot.yaw = eulerdot[2][0]

        # setting gammas for getting rotation derivatives
        gamma1 = VPC.Gamma1
        gamma2 = VPC.Gamma2
        gamma3 = VPC.Jzz / VPC.Jdet
        gamma4 = VPC.Jxz / VPC.Jdet
        gamma5 = (VPC.Jzz - VPC.Jxx) / VPC.Jyy
        gamma6 = VPC.Jxz / VPC.Jyy
        gamma7 = VPC.Gamma7
        gamma8 = VPC.Jxx / VPC.Jdet
        # getting body angular acceleration using equations from beard
        dot.p = (gamma1 * state.p * state.q - gamma2 * state.q * state.r) + (
                gamma3 * forcesMoments.Mx + gamma4 * forcesMoments.Mz)
        dot.q = (gamma5 * state.p * state.r - gamma6 * (state.p ** 2 - state.r ** 2)) + (forcesMoments.My / VPC.Jyy)
        dot.r = (gamma7 * state.p * state.q - gamma1 * state.q * state.r) + (
                gamma4 * forcesMoments.Mx + gamma8 * forcesMoments.Mz)
        return dot

    def ForwardEuler(self, dT, state, dot):
        xt1 = States.vehicleState()
        # shamelessly copied off Max's piazza post
        for k in ["pn", "pe", "pd", "u", "v", "w", "p", "q", "r", "yaw", "pitch", "roll"]:
            xt1.__dict__[k] = state.__dict__[k] + dT * dot.__dict__[k]

        return xt1

    def IntegrateState(self, dT, state, dot):
        newState = States.vehicleState()
        x_t1 = self.ForwardEuler(dT, state, dot)
        for k in ["pn", "pe", "pd", "u", "v", "w", "p", "q", "r"]:
            newState.__dict__[k] = x_t1.__dict__[k]
        R_t1 = mm.multiply(self.Rexp(dT, state, dot), state.R)
        newState.R = R_t1
        eulers = Rotations.dcm2Euler(R_t1)
        newState.yaw = eulers[0]
        newState.pitch = eulers[1]
        newState.roll = eulers[2]
        # keeping derived variables constant
        newState.alpha = state.alpha
        newState.beta = state.beta
        newState.Va = state.Va
        newState.chi = math.atan2(dot.pe, dot.pn)

        return newState
