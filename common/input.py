import json
import math
import os
import pickle
import numpy

from collections import defaultdict

from point import Point, SensorNode, RelayNode, distance


class Constants:
    anpha_1 = 0.65
    S = 0.5
    C = 0.5
    beta_1 = 1.2748 - 0.519 * S - 0.512 * C
    beta_2 = 1.3379 - 0.63 * S - 0.166 * C
    pb = 1.3
    ps = 2.66
    mv = 0.1
    esp_w0 = 80.36
    esp_w8 = 4.9
    t_w = 2
    f = 300
    muy_0 = 12.57 * pow(10, -7)
    muy_r = 12.57 * pow(10, -7)

    @classmethod
    def get_esp_s(cls):
        return pow(1.01 + 0.44 * cls.ps, 2) - 0.0062

    @classmethod
    def get_sigma_eff(cls):
        return (0.0467 + 0.224 * cls.pb - 0.411 * cls.S + 0.6614 * cls.C)

    @classmethod
    def get_esp_fw1(cls):
        return (cls.esp_w8 + (cls.esp_w0 - cls.esp_w8) / (1 + pow(2 * 3.14 * cls.f * cls.t_w, 2)))

    @classmethod
    def get_esp_1_or_0(cls):
        return (1.15 * pow(1 + cls.pb / cls.ps * pow(cls.get_esp_s(), cls.anpha_1) + pow(cls.mv, cls.beta_1) * pow(cls.get_esp_fw1(), cls.anpha_1) - cls.mv, 1 / cls.anpha_1) - 0.68)

    @classmethod
    def get_esp_fw2(cls):
        return (2 * 3.14 * cls.f * cls.t_w * (cls.esp_w0 - cls.esp_w8) / (1 + pow(2 * 3.14 * cls.f * cls.t_w, 2)) + cls.get_sigma_eff() * (cls.ps - cls.pb) / (2 * 3.14 + cls.get_esp_1_or_0() * cls.ps * cls.mv))

    @classmethod
    def get_esp_2(cls):
        return 2 * 3.14 * cls.f * cls.t_w * (cls.esp_w0 - cls.esp_w8) / (1 + pow(2 * 3.14 * cls.f * cls.t_w, 2)) + cls.get_sigma_eff() * (cls.ps - cls.pb) / (2 * 3.14 + cls.get_esp_1_or_0() * cls.ps * cls.mv)

    @classmethod
    def get_anpha(cls):
        return (2 * 3.14 * cls.f * math.sqrt(0.5 * cls.muy_r * cls.muy_0 * pow(cls.get_esp_1_or_0(), 2) * (math.sqrt(1 + pow(cls.get_esp_2() / cls.get_esp_1_or_0(), 2)) - 1)))

    @classmethod
    def get_beta(cls):
        return (2 * 3.14 * cls.f * math.sqrt(0.5 * cls.muy_r * cls.muy_0 * pow(cls.get_esp_1_or_0(), 2) * (math.sqrt(1 + pow(cls.get_esp_2() / cls.get_esp_1_or_0(), 2)) + 1)))


class Input:
    def __init__(self, _W=500, _H=500, _depth=1., _height=10., _num_of_relays=10, _num_of_sensors=50,
                 _sensor_radius=25, _communicate_radius=50, _sensors=None, _relays=None, _sensor_coverage=None,
                 _comm_loss_matrix=None):
        self.W = _W
        self.H = _H
        self.depth = _depth
        self.height = _height
        self.num_of_relays = _num_of_relays
        self.num_of_sensors = _num_of_sensors
        self.sensor_radius = _sensor_radius
        self.communicate_radius = _communicate_radius
        self.relays = _relays
        self.sensors = _sensors
        self.sensor_coverage = _sensor_coverage
        self.comm_loss_matrix = _comm_loss_matrix

        if _sensor_coverage == None:
            self.cal_sensor_coverage()
        if _comm_loss_matrix == None:
            self.cal_comm_loss_matrix()

    def cal_comm_loss_matrix(self):
        comm_loss_matrix = numpy.zeros(
            (self.num_of_sensors, self.num_of_sensors))

        for sn in range(self.num_of_sensors):
            for ss in range(sn+1, self.num_of_sensors):
                d = distance(self.sensors[sn], self.sensors[ss])
                loss = 6.4 + 20 * \
                    math.log(10,d) + 20 * math.log(10,Constants.get_beta()
                                                ) + 8.69 * Constants.get_anpha() * d
                comm_loss_matrix[sn][ss] = loss
                comm_loss_matrix[ss][sn] = loss

        self.comm_loss_matrix = comm_loss_matrix

    def cal_sensor_coverage(self):
        sensor_coverage = numpy.zeros((self.num_of_relays, self.num_of_sensors))        
        R = self.sensor_radius

        for rn in range(self.num_of_relays):
            for sn in range(self.num_of_sensors):
                d = distance(self.sensors[sn], self.relays[rn])
                x_atan = numpy.arctan((self.relays[rn].y-self.relays[sn].y)/(self.relays[rn].x - self.sensors[sn].x))
                z_atan = numpy.arctan((self.relays[rn].z - self.relays[sn].z)/d)

                if (d <= 2*R) and (x_atan >= 0 and x_atan <= 1.57) and (z_atan >= 0 and z_atan <= 1.57):
                    sensor_coverage[rn][sn] = 1
                    # sensor_coverage[sn][rn] = 1
                else:
                    sensor_coverage[rn][sn] = 0
                    # sensor_coverage[sn][rn] = 0


        self.sensor_coverage = sensor_coverage

    @classmethod
    def from_file(cls, path):
        f = open(path)
        d = json.load(f)
        return cls.from_dict(d)

    @classmethod
    def from_dict(cls, d):
        W = d['W']
        H = d['H']
        depth = d['depth']
        height = d['height']
        num_of_relays = d['num_of_relays']
        num_of_sensors = d['num_of_sensors']
        relays = []
        sensors = []
        sensor_radius = 25
        communicate_radius = 50

        for i in range(num_of_sensors):
            d['sensors'][i]['index'] = i
            sensors.append(SensorNode.from_dict(d['sensors'][i]))
        for i in range(num_of_relays):
            d['relays'][i]['index'] = i
            relays.append(RelayNode.from_dict(d['relays'][i]))

        return cls(W, H, depth, height, num_of_relays, num_of_sensors, sensor_radius, communicate_radius, sensors, relays)


if __name__ == "__main__":
    inp = Input.from_file('../data/small_data/ga-dem1_r25_1.in')
    print(numpy.count_nonzero(inp.sensor_coverage > 0))
    
