import VehicleClosedLoopControl as VCLC
from ece163.Containers import States


class PayloadAerodynamicModel:
    payload = States.vehicleState
    def __init__(self, pn=0.0, pe=0.0, pd=0.0, u=0.0, v=0.0, w=0.0):
