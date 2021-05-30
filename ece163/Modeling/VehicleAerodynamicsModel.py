"""VehicleAeroDynamicsModel Class
Models aerodynamics for plane described in VehiclePhysicalConstants

Code Author: Kevin Jesubalan (kjesubal@ucsc.edu)
"""

import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Modeling import WindModel
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC


class VehicleAerodynamicsModel:

    def __init__(self, initialSpeed=VPC.InitialSpeed, initialHeight=VPC.InitialDownPosition):
        # initializing
        self.vehicle = VehicleDynamicsModel.VehicleDynamicsModel()
        self.initialSpeed = initialSpeed
        self.initialHeight = initialHeight
        self.wind = WindModel.WindModel()

        # setting height and speed for later use in reset
        self.vehicle.state.pd = initialHeight
        self.vehicle.state.u = initialSpeed

    def reset(self):
        self.vehicle.reset()
        # resetting height and speed to initials specified in intialization
        self.vehicle.state.pd = self.initialHeight
        self.vehicle.state.u = self.initialSpeed

    # Getters and Setters below pretty standard
    def getVehicleState(self):
        return self.vehicle.getVehicleState()

    def setVehicleState(self, state):
        self.vehicle.setVehicleState(state)

    def getVehicleDynamicsModel(self):
        return self.vehicle

    def getWindModel(self):
        return self.wind

    def SetWindModel(self, windmodel):
        self.wind = windmodel

    # updates vehicle model using control forces inputted and vehicles state
    def Update(self, controls):
        state = self.getVehicleState()
        self.wind.Update()
        forces = self.updateForces(state, controls,self.wind.windstate)
        self.vehicle.Update(forces)

    # creates a gravity force object by rotating it into body frame
    def gravityForces(self, state):
        gravforces = Inputs.forcesMoments()  # create force object for gravity
        # gravity force is simple rotation of gravity vector and then multiplying by mass
        Fg = mm.scalarMultiply(VPC.mass, mm.multiply(state.R, [[0], [0], [VPC.g0]]))
        gravforces.Fx = Fg[0][0]
        gravforces.Fy = Fg[1][0]
        gravforces.Fz = Fg[2][0]
        return gravforces

    def CalculateCoeff_alpha(self, alpha):
        # shamelessly stolen from code provided for hw2
        num = (1 + math.exp(-VPC.M * (alpha - VPC.alpha0)) + math.exp(VPC.M * (alpha + VPC.alpha0)))
        den = (1 + math.exp(-VPC.M * (alpha - VPC.alpha0))) * (1 + math.exp(VPC.M * (alpha + VPC.alpha0)))
        sigma = num / den

        # finding pi * e * A * R
        pieAR = math.pi * (VPC.b ** 2 / VPC.S) * VPC.e
        # blending functions for coefficients of lift, drag, and moment from Gabe Lecture
        C_L = (1 - sigma) * (VPC.CL0 + VPC.CLalpha * alpha) + sigma * (2 * math.sin(alpha) * math.cos(alpha))
        C_D = (1 - sigma) * (VPC.CDp + (VPC.CL0 + VPC.CLalpha * alpha) ** 2 / pieAR) + sigma * (
                2 * (math.sin(alpha) ** 2))
        C_M = VPC.CM0 + VPC.CMalpha * alpha

        return C_L, C_D, C_M

    def aeroForces(self, state):

        (C_L, C_D, C_M) = self.CalculateCoeff_alpha(state.alpha)  # getting coefficients for use in calculations

        rotation_stob = [[math.cos(state.alpha), 0, -math.sin(state.alpha)], [0, 1, 0],  # rotation of stability to body
                         [math.sin(state.alpha), 0, math.cos(state.alpha)]]

        # These functions are in order of how they appear in Beard:
        # (pitch moment, lift force, drag force, roll moment, yaw moment)
        m = 1 / 2 * VPC.rho * state.Va * VPC.S * VPC.c * (
                VPC.CM0 * state.Va + VPC.CMalpha * state.alpha * state.Va + VPC.CMq * VPC.c * state.q / 2)
        Fz = -1 / 2 * VPC.rho * state.Va * VPC.S * (C_L * state.Va + VPC.CLq * VPC.c * state.q / 2)
        Fx = -1 / 2 * VPC.rho * state.Va * VPC.S * (C_D * state.Va + VPC.CDq * VPC.c * state.q / 2)

        Fy = 1 / 2 * VPC.rho * state.Va * VPC.S * (
                VPC.CY0 * state.Va + VPC.CYbeta * state.beta * state.Va + state.p * VPC.CYp * VPC.b / 2 + state.r * VPC.CYr * VPC.b / 2)
        l = 1 / 2 * VPC.rho * state.Va * VPC.S * VPC.b * (
                VPC.Cl0 * state.Va + VPC.Clbeta * state.beta * state.Va + state.p * VPC.Clp * VPC.b / 2 + state.r * VPC.Clr * VPC.b / 2)
        n = 1 / 2 * VPC.rho * state.Va * VPC.S * VPC.b * (
                VPC.Cn0 * state.Va + VPC.Cnbeta * state.beta * state.Va + state.p * VPC.Cnp * VPC.b / 2 + state.r * VPC.Cnr * VPC.b / 2)

        # have to rotate lift and drag since they are in stability
        bodyforces = mm.multiply(rotation_stob, [[Fx], [Fy], [Fz]])

        forces = Inputs.forcesMoments(bodyforces[0][0], bodyforces[1][0], bodyforces[2][0], l, m, n)
        return forces

    def controlForces(self, state, controls):
        # need propeller forces to calculate sum of all control forces
        (propf, propm) = self.CalculatePropForces(state.Va, controls.Throttle)
        # rotation matrix for stability to body
        rotation_stob = [[math.cos(state.alpha), 0, -math.sin(state.alpha)], [0, 1, 0],
                         [math.sin(state.alpha), 0, math.cos(state.alpha)]]

        # These functions are in order of how they appear in Beard:
        # (pitch moment, lift force, drag force, roll moment, yaw moment)
        m = 1 / 2 * VPC.rho * state.Va ** 2 * VPC.S * VPC.c * (VPC.CMdeltaE * controls.Elevator)
        Fz = -1 / 2 * VPC.rho * state.Va ** 2 * VPC.S * (VPC.CLdeltaE * controls.Elevator)
        Fx = -1 / 2 * VPC.rho * state.Va ** 2 * VPC.S * (VPC.CDdeltaE * controls.Elevator)

        Fy = 1 / 2 * VPC.rho * state.Va ** 2 * VPC.S * (
                VPC.CYdeltaA * controls.Aileron + VPC.CYdeltaR * controls.Rudder)
        l = 1 / 2 * VPC.rho * state.Va ** 2 * VPC.S * VPC.b * (
                VPC.CldeltaA * controls.Aileron + VPC.CldeltaR * controls.Rudder)
        n = 1 / 2 * VPC.rho * state.Va ** 2 * VPC.S * VPC.b * (
                VPC.CndeltaA * controls.Aileron + VPC.CndeltaR * controls.Rudder)

        # have to rotate lift and drag since they are in stability
        bodyforces = mm.multiply(rotation_stob, [[Fx], [Fy], [Fz]])

        forces = Inputs.forcesMoments(bodyforces[0][0] + propf, bodyforces[1][0], bodyforces[2][0], l + propm, m, n)
        return forces

    def CalculatePropForces(self, Va, Throttle):
        # a, b, c are variables used for solving quadratic equation for torque as seen in Propeller Cheat Sheet
        a = (VPC.rho * VPC.D_prop ** 5 * VPC.C_Q0) / (4 * math.pi ** 2)
        b = (VPC.rho * VPC.D_prop ** 4 * Va * VPC.C_Q1) / (2 * math.pi) + VPC.KQ ** 2 / VPC.R_motor
        c = VPC.rho * VPC.D_prop ** 3 * Va ** 2 * VPC.C_Q2 - VPC.KQ * VPC.V_max * Throttle / VPC.R_motor + VPC.KQ * VPC.i0

        # solving the quadratic equation using earlier variables
        try:
            torque = (-b + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
        except:
            torque = 100.0
        # getting advance ratio using torque
        J = (2 * math.pi * Va) / (torque * VPC.D_prop)
        # using advance ratio to get coefficients for thrust force and moment
        Ct = VPC.C_T0 + VPC.C_T1 * J + VPC.C_T2 * J ** 2
        Cq = VPC.C_Q0 + VPC.C_Q1 * J + VPC.C_Q2 * J ** 2
        # solving for thrust force and moment
        Fx = (VPC.rho * torque ** 2 * VPC.D_prop ** 4 * Ct) / (4 * math.pi ** 2)
        Mx = (-VPC.rho * torque ** 2 * VPC.D_prop ** 5 * Cq) / (4 * math.pi ** 2)
        return Fx, Mx

    def updateForces(self, state, controls,wind=None):
        # update our windmodel to get windstate
        if wind == None:
            wind = States.windState()
        # need Airspeed, Angle of Attack, and Sideslip Angle from calculateairsspeed()
        state.Va,state.alpha,state.beta = self.CalculateAirspeed(state,wind)

        # use earlier functions to get sum of all forces and moments
        control = self.controlForces(state, controls)
        gravity = self.gravityForces(state)
        aero = self.aeroForces(state)
        forces = Inputs.forcesMoments(control.Fx + gravity.Fx + aero.Fx, control.Fy + gravity.Fy + aero.Fy,
                                      control.Fz + gravity.Fz + aero.Fz, control.Mx + gravity.Mx + aero.Mx,
                                      control.My + gravity.My + aero.My, control.Mz + gravity.Mz + aero.Mz)
        return forces

    def CalculateAirspeed(self, state, wind):
        # get stead Wind and azimuth and elevator angles
        Ws = math.hypot(wind.Wn ,  wind.We  , wind.Wd )
        azi = math.atan2(wind.We, wind.Wn)
        # in case Ws is zero set ele 0
        if math.isclose(Ws, 0.0):
            ele = 0
        else:
            ele = -math.asin(wind.Wd / Ws)

        # rotation matrix for azimuth-elevator
        rot_azi = [[math.cos(azi), math.sin(azi), 0], [-math.sin(azi), math.cos(azi), 0], [0, 0, 1]]
        rot_ele = [[math.cos(ele),0,-math.sin(ele)], [0,1,0], [math.sin(ele),0,math.cos(ele)]]
        rot_aziele = mm.multiply(rot_ele, rot_azi)

        # getting wind vector
        Wb = mm.multiply(state.R,mm.add([[wind.Wn],[wind.We],[wind.Wd]],mm.multiply(mm.transpose(rot_aziele),[[wind.Wu],[wind.Wv],[wind.Ww]])))
        #getting airspeed vector by subtracting state speed vector from wind vector
        airspeed = mm.subtract([[state.u], [state.v], [state.w]],[[Wb[0][0]],[Wb[1][0]],[Wb[2][0]]])
        #finding Va, alpha, beta
        Va = math.hypot(airspeed[0][0],airspeed[1][0],airspeed[2][0])
        alpha = math.atan2(airspeed[2][0],airspeed[0][0])
        if math.isclose(Va, 0.0):
            beta = 0.0
        else:
            beta = math.asin(airspeed[1][0] / Va)

        return Va, alpha, beta
