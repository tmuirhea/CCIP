'''
File that contains the machinery to calculate the gains for successive loop closure for both lateral and longitudinal modes of the UAV

Code Author: Kevin Jesubalan (kjesubal@ucsc.edu)
'''

import math
import pickle
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Containers import Linearized



def computeGains(tuningParameters=Controls.controlTuning(), linearizedModel=Linearized.transferFunctions()):
    # Lateral Gains
    kp_roll = tuningParameters.Wn_roll ** 2 / linearizedModel.a_phi2
    kd_roll = (
                      2 * tuningParameters.Zeta_roll * tuningParameters.Wn_roll - linearizedModel.a_phi1) / linearizedModel.a_phi2
    ki_roll = 0.001
    kp_sideslip = (
                          2 * tuningParameters.Zeta_sideslip * tuningParameters.Wn_sideslip - linearizedModel.a_beta1) / linearizedModel.a_beta2
    ki_sideslip = tuningParameters.Wn_sideslip ** 2 / linearizedModel.a_beta2
    kp_course = 2 * tuningParameters.Zeta_course * tuningParameters.Wn_course * linearizedModel.Va_trim / VPC.g0
    ki_course = tuningParameters.Wn_course ** 2 * linearizedModel.Va_trim / VPC.g0
    # Longitudinal Gains
    kp_pitch = (tuningParameters.Wn_pitch ** 2 - linearizedModel.a_theta2) / linearizedModel.a_theta3
    kd_pitch = (
                       2 * tuningParameters.Wn_pitch * tuningParameters.Zeta_pitch - linearizedModel.a_theta1) / linearizedModel.a_theta3
    KthetaDC = kp_pitch * linearizedModel.a_theta3 / (linearizedModel.a_theta2 + kp_pitch * linearizedModel.a_theta3)
    kp_altitude = (2 * tuningParameters.Zeta_altitude * tuningParameters.Wn_altitude) / (
            KthetaDC * linearizedModel.Va_trim)
    ki_altitude = (tuningParameters.Wn_altitude ** 2) / (KthetaDC * linearizedModel.Va_trim)
    ki_SpeedfromThrottle = tuningParameters.Wn_SpeedfromThrottle ** 2 / linearizedModel.a_V2
    kp_SpeedfromThrottle = (
                                   2 * tuningParameters.Zeta_SpeedfromThrottle * tuningParameters.Wn_SpeedfromThrottle - linearizedModel.a_V1) / linearizedModel.a_V2
    kp_SpeedfromElevator = (
                                   linearizedModel.a_V1 - 2 * tuningParameters.Zeta_SpeedfromElevator * tuningParameters.Wn_SpeedfromElevator) / (
                                   KthetaDC * VPC.g0)
    ki_SpeedfromElevator = -(tuningParameters.Wn_SpeedfromElevator ** 2) / (KthetaDC * VPC.g0)
    return Controls.controlGains(kp_roll, kd_roll, ki_roll, kp_sideslip, ki_sideslip, kp_course, ki_course, kp_pitch,
                                 kd_pitch, kp_altitude, ki_altitude, kp_SpeedfromThrottle, ki_SpeedfromThrottle,
                                 kp_SpeedfromElevator, ki_SpeedfromElevator)


def computeTuningParameters(controlGains=Controls.controlGains(), linearizedModel=Linearized.transferFunctions()):
    try:
        # tuning knobs for lateral control (ignoring Ki_phi)
        Wn_roll = math.sqrt(controlGains.kp_roll * linearizedModel.a_phi2)
        Zeta_roll = (linearizedModel.a_phi1 + linearizedModel.a_phi2 * controlGains.kd_roll) / (2 * Wn_roll)
        Wn_course = math.sqrt(VPC.g0 * controlGains.ki_course / linearizedModel.Va_trim)
        Zeta_course = (VPC.g0 * controlGains.kp_course) / (linearizedModel.Va_trim * Wn_course  * 2)
        Wn_sideslip = math.sqrt(linearizedModel.a_beta2 * controlGains.ki_sideslip)
        Zeta_sideslip = (linearizedModel.a_beta1 + linearizedModel.a_beta2 * controlGains.kp_sideslip) / (
                2 * Wn_sideslip)
        # tuning knobs for longitudinal control
        Wn_pitch = math.sqrt(linearizedModel.a_theta2 + controlGains.kp_pitch * linearizedModel.a_theta3)
        Zeta_pitch = (linearizedModel.a_theta1 + controlGains.kd_pitch * linearizedModel.a_theta3) / (2 * Wn_pitch)

        KthetaDC = controlGains.kp_pitch * linearizedModel.a_theta3 / (
                linearizedModel.a_theta2 + controlGains.kp_pitch * linearizedModel.a_theta3)
        Wn_altitude = math.sqrt(KthetaDC * linearizedModel.Va_trim * controlGains.ki_altitude)
        Zeta_altitude = (KthetaDC * linearizedModel.Va_trim * controlGains.kp_altitude) / (2 * Wn_altitude)
        Wn_SpeedfromThrottle = math.sqrt(linearizedModel.a_V2 * controlGains.ki_SpeedfromThrottle)
        Zeta_SpeedfromThrottle = (linearizedModel.a_V1 + linearizedModel.a_V2 * controlGains.kp_SpeedfromThrottle) / (
                2 * Wn_SpeedfromThrottle)
        Wn_SpeedfromElevator = math.sqrt(-KthetaDC * VPC.g0 * controlGains.ki_SpeedfromElevator)
        Zeta_SpeedfromElevator = (linearizedModel.a_V1 - KthetaDC * VPC.g0 * controlGains.kp_SpeedfromElevator) / (
                2 * Wn_SpeedfromElevator)
        return Controls.controlTuning(Wn_roll, Zeta_roll, Wn_course, Zeta_course, Wn_sideslip, Zeta_sideslip, Wn_pitch,
                                      Zeta_pitch, Wn_altitude, Zeta_altitude, Wn_SpeedfromThrottle,
                                      Zeta_SpeedfromThrottle,
                                      Wn_SpeedfromElevator, Zeta_SpeedfromElevator)
    except ValueError:
        return Controls.controlTuning()
