"""This file contains multiple classes
    The VehicleClosedLoopControl class
    and various control loop classes PID,PD,and PI
Uses all of these to implement closed loop control as described in Beard Ch 6

Code Author: Kevin Jesubalan (kjesubal@ucsc.edu)
"""
import math
import sys
import ece163.Containers.Inputs as Inputs
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModule


# will be using inheritance to simplify certain things hopefully
class PControl:
    def __init__(self, kp=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp = kp
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit

    def Update(self, command=0.0, current=0.0):
        u = (command - current) * self.kp + self.trim
        return u

    def setPGains(self, kp=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp = kp
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit


class PDControl(PControl):
    def __init__(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        super().__init__(kp, trim, lowLimit, highLimit)
        self.kd = kd

    # calculating actuator input using derivative value and error value
    def Update(self, command=0.0, current=0.0, derivative=0.0):
        u = super().Update(command, current) - self.kd * derivative
        if u < self.lowLimit:
            u = self.lowLimit
        elif u > self.highLimit:
            u = self.highLimit
        return u

    def setPDGains(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        super().setPGains(kp, trim, lowLimit, highLimit)
        self.kd = kd


class PIControl(PControl):
    def __init__(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        super().__init__(kp, trim, lowLimit, highLimit)
        self.dT = dT
        self.ki = ki
        self.error = 0
        self.accumulator = 0

    def Update(self, command=0.0, current=0.0):
        # solving for accumulation parameter
        accumulation = self.accumulator + self.dT * ((command - current) + self.error) / 2
        # solving for the actuator input
        u = super().Update(command, current) + accumulation * self.ki
        # doings checks for if u is in need of saturation and if so not to accumulate
        if u < self.lowLimit:
            u = self.lowLimit
        elif u > self.highLimit:
            u = self.highLimit
        else:
            self.accumulator = accumulation

        # getting error for accumulation later
        self.error = (command - current)
        return u

    def resetIntegrator(self):
        self.accumulator = 0
        self.error = 0

    def setPIGains(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        super().setPGains(kp, trim, lowLimit, highLimit)
        self.dT = dT
        self.ki = ki
        self.resetIntegrator()


class PIDControl(PIControl):
    def __init__(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        super().__init__(dT, kp, ki, trim, lowLimit, highLimit)
        self.dT = dT
        self.ki = ki
        self.kd = kd

    # Update in PID is same as in PI but also includes a derivative value
    def Update(self, command=0.0, current=0.0, derivative=0.0):
        accumulation = self.accumulator + self.dT * ((command - current) + self.error) / 2
        u = (command - current) * self.kp + self.trim + accumulation * self.ki - self.kd * derivative
        if u < self.lowLimit:
            u = self.lowLimit
        elif u > self.highLimit:
            u = self.highLimit
        else:
            self.accumulator = accumulation
        self.error = (command - current)
        return u

    def setPIDGains(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        super().setPIGains(dT, kp, ki, trim, lowLimit, highLimit)
        self.kd = kd


class VehicleClosedLoopControl:
    def __init__(self, dT=VPC.dT, rudderControlSource='SIDESLIP'):
        # variables needed for Closed Loop Control
        self.VAM = VehicleAerodynamicsModule.VehicleAerodynamicsModel()
        self.gainz = Controls.controlGains()
        self.triminputs = Inputs.controlInputs()
        self.VAMinputs = Inputs.controlInputs()

        # used for current mode clarity
        self.states = ["CLIMBING", "HOLDING", "DESCENDING"]
        self.mode = 1

        # all P{something} controls
        self.rollFromCourse = PIControl()
        self.rudderFromSideslip = PIControl()
        self.throttleFromAirspeed = PIControl()
        self.pitchFromAltitude = PIControl()
        self.pitchFromAirspeed = PIControl()
        self.elevatorFromPitch = PDControl()
        self.aileronFromRoll = PIDControl()

    def Update(self, referenceCommands=Controls.referenceCommands()):
        self.VAMinputs = self.UpdateControlCommands(referenceCommands, self.VAM.getVehicleState())
        self.VAM.Update(self.VAMinputs)

    def UpdateControlCommands(self, referenceCommands, state):
        # needed threshholds for checking mode
        upper_thresh = referenceCommands.commandedAltitude + VPC.altitudeHoldZone
        lower_thresh = referenceCommands.commandedAltitude - VPC.altitudeHoldZone
        # check for whether in correct mode
        if lower_thresh < -state.pd < upper_thresh and self.states[self.mode] == "DESCENDING":
            self.pitchFromAltitude.resetIntegrator()
            self.mode = 1
        elif -state.pd < lower_thresh and self.states[self.mode] == "DESCENDING":
            self.mode = 0
            self.pitchFromAltitude.resetIntegrator()
            self.pitchFromAirspeed.resetIntegrator()
        elif -state.pd > upper_thresh and self.states[self.mode] == "HOLDING":
            self.mode = 2
            self.pitchFromAirspeed.resetIntegrator()
        elif -state.pd < lower_thresh and self.states[self.mode] == "HOLDING":
            self.mode = 0
            self.pitchFromAirspeed.resetIntegrator()
        elif lower_thresh < -state.pd < upper_thresh and self.states[self.mode] == "CLIMBING":
            self.mode = 1
            self.pitchFromAltitude.resetIntegrator()
        elif -state.pd > upper_thresh and self.states[self.mode] == "CLIMBING":
            self.mode = 2
            self.pitchFromAltitude.resetIntegrator()
            self.pitchFromAirspeed.resetIntegrator()

        # getting aileroncommand
        chierror = referenceCommands.commandedCourse - state.chi
        if chierror >= math.pi:
            state.chi += 2 * math.pi
        elif chierror <= -math.pi:
            state.chi -= 2 * math.pi
        referenceCommands.commandedRoll = self.rollFromCourse.Update(referenceCommands.commandedCourse, state.chi)
        aileroncommand = self.aileronFromRoll.Update(referenceCommands.commandedRoll, state.roll, state.p)

        # getting ruddercommand
        ruddercommand = self.rudderFromSideslip.Update(0, state.beta)

        # check mode for determining throttlecommand and pitch command
        if self.states[self.mode] == "CLIMBING":
            # pitchCommand determined by airspeed,throttleCommand = VPC.maxControls.Throttle
            throttlecommand = VPC.maxControls.Throttle
            referenceCommands.commandedPitch = self.pitchFromAirspeed.Update(referenceCommands.commandedAirspeed,
                                                                             state.Va)
        elif self.states[self.mode] == "HOLDING":
            # pitchCommand determined by altitude, throttleCommand determined by airspeed
            throttlecommand = self.throttleFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
            referenceCommands.commandedPitch = self.pitchFromAltitude.Update(referenceCommands.commandedAltitude,
                                                                             -state.pd)

        elif self.states[self.mode] == "DESCENDING":
            # pitchCommand determined by airspeed, throttleCommand = VPC.minControls.Throttle
            throttlecommand = VPC.minControls.Throttle
            referenceCommands.commandedPitch = self.pitchFromAirspeed.Update(referenceCommands.commandedAirspeed,
                                                                             state.Va)
        else:
            raise Exception("Invalid state")

        # getting elevatorcommand
        elevatorcommand = self.elevatorFromPitch.Update(referenceCommands.commandedPitch, state.pitch, state.q)

        return Inputs.controlInputs(throttlecommand, aileroncommand, elevatorcommand, ruddercommand)

    # some simple getters
    def getControlGains(self):
        return self.gainz

    def getTrimInputs(self):
        return self.triminputs

    def getVehicleAerodynamicsModel(self):
        return self.VAM

    def getVehicleControlSurfaces(self):
        return self.VAMinputs

    def getVehicleState(self):
        return self.VAM.getVehicleState()

    # resets Vehicle Model and integrators
    def reset(self):
        self.rollFromCourse.resetIntegrator()
        self.rudderFromSideslip.resetIntegrator()
        self.throttleFromAirspeed.resetIntegrator()
        self.pitchFromAltitude.resetIntegrator()
        self.pitchFromAirspeed.resetIntegrator()
        self.aileronFromRoll.resetIntegrator()
        self.VAM.reset()

    # sets gain values using a controlGains input and various constants
    def setControlGains(self, controlGains=Controls.controlGains()):
        self.rollFromCourse.setPIGains(VPC.dT, controlGains.kp_course, controlGains.ki_course, 0,
                                       math.radians(-VPC.bankAngleLimit),
                                       math.radians(VPC.bankAngleLimit))

        self.rudderFromSideslip.setPIGains(VPC.dT, controlGains.kp_sideslip, controlGains.ki_sideslip,
                                           self.triminputs.Rudder, VPC.minControls.Rudder, VPC.maxControls.Rudder)

        self.throttleFromAirspeed.setPIGains(VPC.dT, controlGains.kp_SpeedfromThrottle,
                                             controlGains.ki_SpeedfromThrottle, self.triminputs.Throttle,
                                             VPC.minControls.Throttle,
                                             VPC.maxControls.Throttle)

        self.pitchFromAltitude.setPIGains(VPC.dT, controlGains.kp_altitude, controlGains.ki_altitude, 0,
                                          math.radians(-VPC.pitchAngleLimit), math.radians(VPC.pitchAngleLimit))
        self.pitchFromAirspeed.setPIGains(VPC.dT, controlGains.kp_SpeedfromElevator, controlGains.ki_SpeedfromElevator,
                                          0, math.radians(-VPC.pitchAngleLimit), math.radians(VPC.pitchAngleLimit))

        self.elevatorFromPitch.setPDGains(controlGains.kp_pitch, controlGains.kd_pitch, self.triminputs.Elevator,
                                          VPC.minControls.Elevator, VPC.maxControls.Elevator)

        self.aileronFromRoll.setPIDGains(VPC.dT, controlGains.kp_roll, controlGains.kd_roll, controlGains.ki_roll,
                                         self.triminputs.Aileron, VPC.minControls.Aileron, VPC.maxControls.Aileron)

    # some setters below
    def setTrimInputs(self, trimInputs=Inputs.controlInputs(Throttle=0.5, Aileron=0.0, Elevator=0.0, Rudder=0.0)):
        self.triminputs = trimInputs

    def setVehicleState(self, state):
        self.VAM.setVehicleState(state)
