"""SensorModel Class
SensorModel class which models the various sensors used in our simulations
(Accelerometer, Gyro, Gps, Magnometer)
Also has GaussMarkov class for modeling noise that sensors would see in real life

Code Author: Kevin Jesubalan (kjesubal@ucsc.edu)
"""
import math
import random
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Utilities import MatrixMath as mm
from ..Containers import Sensors
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleAerodynamicsModel


# Gauss Markov class which will handle doing the Gauss Markov process with a v state variable
class GaussMarkov:
    def __init__(self, dT=VPC.dT, tau=1e6, eta=0.0):
        self.dT = dT
        self.tau = tau
        self.eta = eta
        self.v = 0

    # resets state to 0
    def reset(self):
        self.v = 0

    # does the gauss-markov process updating internal state v
    def update(self, vnoise=None):
        if vnoise is None:
            self.v = math.exp(-self.dT / self.tau) * self.v + random.gauss(0, self.eta)
        else:
            self.v = math.exp(-self.dT / self.tau) * self.v + vnoise
        return self.v


# Class that handles doing 3 GaussMarkov processes at once. All functions just are basically 3 instances of GaussMarkov
class GaussMarkovXYZ:
    def __init__(self, dT=VPC.dT, tauX=1e6, etaX=0.0, tauY=None, etaY=None, tauZ=None, etaZ=None):
        if tauZ is not None:
            self.gaussX = GaussMarkov(dT, tauX, etaX)
            self.gaussY = GaussMarkov(dT, tauY, etaY)
            self.gaussZ = GaussMarkov(dT, tauZ, etaZ)

        elif tauY is not None:
            self.gaussX = GaussMarkov(dT, tauX, etaX)
            self.gaussY = GaussMarkov(dT, tauY, etaY)
            self.gaussZ = GaussMarkov(dT, tauY, etaY)
        else:
            self.gaussX = GaussMarkov(dT, tauX, etaX)
            self.gaussY = GaussMarkov(dT, tauX, etaX)
            self.gaussZ = GaussMarkov(dT, tauX, etaX)

    def reset(self):
        self.gaussX.reset()
        self.gaussY.reset()
        self.gaussZ.reset()

    def update(self, vXnoise=None, vYnoise=None, vZnoise=None):
        vX = self.gaussX.update(vXnoise)
        vY = self.gaussY.update(vYnoise)
        vZ = self.gaussZ.update(vZnoise)
        return vX, vY, vZ


# class which contains all the sensor functions
class SensorsModel:
    def __init__(self, aeroModel=VehicleAerodynamicsModel.VehicleAerodynamicsModel(), taugyro=VSC.gyro_tau,
                 etagyro=VSC.gyro_eta, tauGPS=VSC.GPS_tau, etaGPSHorizontal=VSC.GPS_etaHorizontal,
                 etaGPSVertical=VSC.GPS_etaVertical, gpsUpdateHz=VSC.GPS_rate):

        self.vehicle = aeroModel  # vehicle variable
        self.dT = aeroModel.vehicle.dT  # dT comes from the vehicle
        # Creating vehicleSensor containers to contain true values, noisy values, biases, and sigmas for each sensor
        self.trueSensors = Sensors.vehicleSensors()
        self.noisySensors = Sensors.vehicleSensors()
        self.bias = Sensors.vehicleSensors()
        self.sigma = Sensors.vehicleSensors()

        # create tick variables for calculating when to update GPS
        self.tickCounter = 0  # counter variable
        self.gpsTickUpdate = 1 / gpsUpdateHz / self.dT  # find number of dTs in one period of GPS update cycle
        # last two variables for creating the gauss-markovxyz variables for gyro and GPS since they drift over time
        self.gyroDrift = GaussMarkovXYZ(self.dT, taugyro, etagyro)
        self.gpsDrift = GaussMarkovXYZ(self.dT, tauGPS, etaGPSVertical, tauGPS, etaGPSHorizontal)
        # Intiialize Biases and Sigmas
        self.initializeBiases()
        self.initializeSigmas()

    # getters
    def getSensorsNoisy(self):
        return self.noisySensors

    def getSensorsTrue(self):
        return self.trueSensors

    # uses biases given to create random biases in the range of [-bias,bias]. GPS bias is 0
    def initializeBiases(self, gyroBias=VSC.gyro_bias, accelBias=VSC.accel_bias, magBias=VSC.mag_bias,
                         baroBias=VSC.baro_bias, pitotBias=VSC.pitot_bias):
        self.bias.gyro_x = random.uniform(-gyroBias, gyroBias)
        self.bias.gyro_y = random.uniform(-gyroBias, gyroBias)
        self.bias.gyro_z = random.uniform(-gyroBias, gyroBias)
        self.bias.accel_x = random.uniform(-accelBias, accelBias)
        self.bias.accel_y = random.uniform(-accelBias, accelBias)
        self.bias.accel_z = random.uniform(-accelBias, accelBias)
        self.bias.mag_x = random.uniform(-magBias, magBias)
        self.bias.mag_y = random.uniform(-magBias, magBias)
        self.bias.mag_z = random.uniform(-magBias, magBias)
        self.bias.baro = random.uniform(-baroBias, baroBias)
        self.bias.pitot = random.uniform(-pitotBias, pitotBias)
        self.bias.gps_sog = self.bias.gps_e = self.bias.gps_n = self.bias.gps_alt = self.bias.gps_cog = 0

    # initialize sigma variable from ones passed
    def initializeSigmas(self, gyroSigma=VSC.gyro_sigma, accelSigma=VSC.accel_sigma, magSigma=VSC.mag_sigma,
                         baroSigma=VSC.baro_sigma, pitotSigma=VSC.pitot_sigma,
                         gpsSigmaHorizontal=VSC.GPS_sigmaHorizontal, gpsSigmaVertical=VSC.GPS_sigmaVertical,
                         gpsSigmaSOG=VSC.GPS_sigmaSOG, gpsSigmaCOG=VSC.GPS_sigmaCOG):
        self.sigma.gyro_x = self.sigma.gyro_y = self.sigma.gyro_z = gyroSigma

        self.sigma.accel_x = self.sigma.accel_y = self.sigma.accel_z = accelSigma

        self.sigma.mag_x = self.sigma.mag_y = self.sigma.mag_z = magSigma

        self.sigma.baro = baroSigma
        self.sigma.pitot = pitotSigma
        self.sigma.gps_sog = gpsSigmaSOG
        self.sigma.gps_cog = gpsSigmaCOG
        self.sigma.gps_n = self.sigma.gps_e = gpsSigmaHorizontal
        self.sigma.gps_alt = gpsSigmaVertical

    # reset all the sensor data and the Gauss Markov processes. Also re-initializes biases
    def reset(self):
        self.trueSensors = Sensors.vehicleSensors()
        self.noisySensors = Sensors.vehicleSensors()
        self.initializeBiases()
        self.gyroDrift.reset()
        self.gpsDrift.reset()

    # quite simply calculates noisy and true sensor data then updates the tickcounter by 1
    def update(self):
        self.trueSensors = self.updateSensorsTrue(self.trueSensors, self.vehicle.vehicle.state,
                                                  self.vehicle.vehicle.dot)
        self.noisySensors = self.updateSensorsNoisy(self.trueSensors, self.noisySensors, self.bias, self.sigma)
        self.tickCounter += 1

    # equations for accelerometer true taken from Gabe lecture notes
    def updateAccelsTrue(self, state, dot):
        ax = dot.u + state.q * state.w - state.r * state.v + VPC.g0 * math.sin(state.pitch)
        ay = dot.v + state.r * state.u - state.p * state.w - VPC.g0 * math.cos(state.pitch) * math.sin(
            state.roll)
        az = dot.w + state.p * state.v - state.q * state.u - VPC.g0 * math.cos(state.pitch) * math.cos(
            state.roll)
        return ax, ay, az

    # equations for GPS true taken from Gabe lecture notes
    def updateGPSTrue(self, state, dot):
        Vg = math.sqrt(dot.pe ** 2 + dot.pn ** 2)
        gpsCourse = math.atan2(dot.pe, dot.pn)
        yGPSn = state.pn
        yGPSe = state.pe
        yGPSh = -state.pd
        return yGPSn, yGPSe, yGPSh, Vg, gpsCourse

    # equations for Gyro true taken from Gabe lecture notes
    def updateGyrosTrue(self, state):
        gx = state.p
        gy = state.q
        gz = state.r
        return gx, gy, gz

    # equations for Magnometer true taken from Gabe lecture notes
    def updateMagsTrue(self, state):
        mags = mm.multiply(state.R, VSC.magfield)
        return mags[0][0], mags[1][0], mags[2][0]

    # equations for Pressure Sensors true taken from Gabe lecture notes
    def updatePressureSensorsTrue(self, state):
        baro = VSC.Pground - VPC.rho * VPC.g0 * -state.pd
        pitot = VPC.rho * state.Va ** 2 / 2
        return baro, pitot

    # Adds noise to every sensor with the initial bias and then a random gaussian noise
    # GPS has no initial bias and both it and Gyro have a guass markov drift noise as well
    def updateSensorsNoisy(self, trueSensors=Sensors.vehicleSensors(), noisySensors=Sensors.vehicleSensors(),
                           sensorBiases=Sensors.vehicleSensors(), sensorSigmas=Sensors.vehicleSensors()):
        noise = Sensors.vehicleSensors()
        # this if checks whether the counter says its time to update GPS otherwise there's a zero order hold
        if self.tickCounter % self.gpsTickUpdate == 0:
            gpsN, gpsE, gpsAlt = self.gyroDrift.update()
            noise.gps_n = trueSensors.gps_n + gpsN + random.gauss(0, sensorSigmas.gps_n)
            noise.gps_e = trueSensors.gps_e + gpsE + random.gauss(0, sensorSigmas.gps_e)
            noise.gps_alt = trueSensors.gps_alt + gpsAlt + random.gauss(0, sensorSigmas.gps_alt)
            noise.gps_sog = trueSensors.gps_sog + random.gauss(0, sensorSigmas.gps_sog)
            # this is to avoid divide by zero error
            if trueSensors.gps_sog == 0:
                noise.gps_cog = trueSensors.gps_cog + random.gauss(0, sensorSigmas.gps_cog)
            else:
                noise.gps_cog = trueSensors.gps_cog + random.gauss(0, sensorSigmas.gps_cog) * (
                            VPC.InitialSpeed / trueSensors.gps_sog)
            # restricting cog to [-pi to pi] since noise could potentially push it out of that range
            if noise.gps_cog > math.pi:
                noise.gps_cog = noise.gps_cog * (math.pi / noise.gps_cog)
            elif noise.gps_cog < -math.pi:
                noise.gps_cog = noise.gps_cog * (-math.pi / noise.gps_cog)
            self.tickCounter = 0
        # if it's not time to update just uses previous values
        else:
            noise.gps_n = noisySensors.gps_n
            noise.gps_e = noisySensors.gps_e
            noise.gps_alt = noisySensors.gps_alt
            noise.gps_sog = noisySensors.gps_sog
            noise.gps_cog = noisySensors.gps_cog
        # gyro needs it's Gauss Markov time drift function updated for its noise
        gyroX, gyroY, gyroZ = self.gyroDrift.update()

        noise.gyro_x = trueSensors.gyro_x + random.gauss(0, sensorSigmas.gyro_x) + sensorBiases.gyro_x + gyroX
        noise.gyro_y = trueSensors.gyro_x + random.gauss(0, sensorSigmas.gyro_x) + sensorBiases.gyro_x + gyroY
        noise.gyro_z = trueSensors.gyro_x + random.gauss(0, sensorSigmas.gyro_x) + sensorBiases.gyro_x + gyroZ

        # rest of the sensors just add the gaussian white noise to a initial bias
        noise.accel_x = trueSensors.accel_x + sensorBiases.accel_x + random.gauss(0, sensorSigmas.accel_x)
        noise.accel_y = trueSensors.accel_y + sensorBiases.accel_y + random.gauss(0, sensorSigmas.accel_y)
        noise.accel_z = trueSensors.accel_z + sensorBiases.accel_z + random.gauss(0, sensorSigmas.accel_z)

        noise.mag_x = trueSensors.mag_x + sensorBiases.mag_x + random.gauss(0, sensorSigmas.mag_x)
        noise.mag_y = trueSensors.mag_y + sensorBiases.mag_y + random.gauss(0, sensorSigmas.mag_y)
        noise.mag_z = trueSensors.mag_z + sensorBiases.mag_z + random.gauss(0, sensorSigmas.mag_z)

        noise.baro = trueSensors.baro + sensorBiases.baro + random.gauss(0, sensorSigmas.baro)
        noise.pitot = trueSensors.pitot + sensorBiases.pitot + random.gauss(0, sensorSigmas.pitot)
        return noise

    # just adds all the true sensor data together. Also checks whether it's time to update GPS or not
    def updateSensorsTrue(self, prevTrueSensors, state, dot):
        sensor = Sensors.vehicleSensors()

        # this if checks whether the counter says its time to update GPS otherwise there's a zero order hold
        if self.tickCounter % self.gpsTickUpdate == 0:
            sensor.gps_n, sensor.gps_e, sensor.gps_alt, sensor.gps_sog, sensor.gps_cog = self.updateGPSTrue(state, dot)

        # if it's not time to update just uses previous values
        else:
            sensor.gps_n = prevTrueSensors.gps_n
            sensor.gps_e = prevTrueSensors.gps_e
            sensor.gps_alt = prevTrueSensors.gps_alt
            sensor.gps_sog = prevTrueSensors.gps_sog
            sensor.gps_cog = prevTrueSensors.gps_cog

        # rest of sensors just call their respective functions
        sensor.baro, sensor.pitot = self.updatePressureSensorsTrue(state)
        sensor.mag_x, sensor.mag_y, sensor.mag_z = self.updateMagsTrue(state)
        sensor.accel_x, sensor.accel_y, sensor.accel_z = self.updateAccelsTrue(state, dot)
        sensor.gyro_x, sensor.gyro_y, sensor.gyro_z = self.updateGyrosTrue(state)

        return sensor
