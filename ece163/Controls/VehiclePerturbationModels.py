import math
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath as mm


def CreateTransferFunction(trimState, trimInputs):
    tf = Linearized.transferFunctions()
    tf.Va_trim = trimState.Va
    tf.alpha_trim = trimState.alpha
    tf.beta_trim = trimState.beta
    tf.gamma_trim = trimState.pitch - trimState.alpha
    tf.phi_trim = trimState.roll
    tf.theta_trim = trimState.pitch
    tf.a_phi1 = -VPC.rho / 2 * trimState.Va ** 2 * VPC.S * VPC.b * VPC.Cpp * VPC.b / 2 / trimState.Va
    tf.a_phi2 = VPC.rho / 2 * trimState.Va ** 2 * VPC.S * VPC.b * VPC.CpdeltaA
    tf.a_beta1 = -VPC.rho / 2 * trimState.Va * VPC.S * VPC.CYbeta / VPC.mass
    tf.a_beta2 = VPC.rho / 2 * trimState.Va * VPC.S * VPC.CYdeltaR / VPC.mass
    tf.a_theta1 = -VPC.rho / 2 * trimState.Va ** 2 * VPC.c * VPC.S / 2 * VPC.CMq / VPC.Jyy * VPC.c / trimState.Va
    tf.a_theta2 = -VPC.rho / 2 * trimState.Va ** 2 * VPC.c * VPC.S * VPC.CMalpha / VPC.Jyy
    tf.a_theta3 = VPC.rho / 2 * trimState.Va ** 2 * VPC.c * VPC.S * VPC.CMdeltaE / VPC.Jyy
    tf.a_V1 = (VPC.rho * tf.Va_trim * VPC.S / VPC.mass) * (VPC.CD0 + VPC.CDalpha * tf.alpha_trim + VPC.CDdeltaE * trimInputs.Elevator) - dThrust_dVa(tf.Va_trim,trimInputs.Throttle)/VPC.mass
    tf.a_V2 = dThrust_dThrottle(tf.Va_trim,trimInputs.Throttle)/VPC.mass
    tf.a_V3 = VPC.g0 * math.cos(tf.theta_trim - tf.alpha_trim)
    return tf


def dThrust_dVa(Va, Throttle, epsilon=0.5):
    return (VehicleAerodynamicsModel.VehicleAerodynamicsModel.CalculatePropForces(None,Va + epsilon, Throttle)[0] -
            VehicleAerodynamicsModel.VehicleAerodynamicsModel.CalculatePropForces(None,Va, Throttle)[0]) / epsilon


def dThrust_dThrottle(Va, Throttle, epsilon=0.01):
    return (VehicleAerodynamicsModel.VehicleAerodynamicsModel.CalculatePropForces(None,Va, Throttle + epsilon)[0] -
            VehicleAerodynamicsModel.VehicleAerodynamicsModel.CalculatePropForces(None,Va, Throttle)[0]) / epsilon
