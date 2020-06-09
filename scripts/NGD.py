import serial
from math import *
import numpy as np

class CalculatePosition:
    def __init__(self):
        #self.Serialdata
        self.avg_resultx = 0.01
        self.avg_resulty = 0.01
        self.avg_resultz = 1
        self.anchor_point = [[0.01, 0.01, 1.6],
                        [0.01, 7.2, 1.6],
                        [4.05, 0.01, 1.6],
                        [4.05, 7.2, 1.6],
                        [6, 0.01, 1.6],
                        [6, 7.2, 1.6]]
        self.boundaryXmax = 19.01
        self.boundaryXmin = 0.01
        self.boundaryYmax = 7.05
        self.boundaryYmin = 0.01

        self.list_resultx = [0, 0, 0, 0, 0]
        self.list_resulty = [0, 0, 0, 0, 0]
    def SumValues(self, SerialString):
        ANCHORx = 0
        ANCHORy = 1
        ANCHORz = 2
        accuracy = 0.02
        stepSize = 0.1
        momentumSize = 0.5
        Serialdata = list(map(float, SerialString.split(',')))
        anchor_cnt = int(len(Serialdata) / 2)
        d = [0, 0, 0, 0, 0, 0]
        resultx = 0
        resulty = 0
        anchor_index = []
        anchor_dist = []
        vdx = 0
        vdy = 0
        for cn in range(anchor_cnt):
            anchor_index.append(int(Serialdata[2 * cn])-1)
            anchor_dist.append(Serialdata[2 * cn + 1])
        for cn in range(anchor_cnt):
            anchor_dist[cn] = abs(pow(anchor_dist[cn], 2) - pow(self.avg_resultz - self.anchor_point[cn][ANCHORz], 2))
        for i in range(100):
            Jdx = 0
            Jdy = 0
            sum_cost = 0
            for k in range(anchor_cnt):
                d[k] = sqrt(pow(self.anchor_point[k][ANCHORx] - (resultx - momentumSize * vdx), 2)
                            + pow(self.anchor_point[k][ANCHORy] - (resulty - momentumSize * vdy), 2))
                Jdx += (self.anchor_point[anchor_index[k]][ANCHORx] - (resultx - momentumSize * vdx)) * (d[k] - anchor_dist[k]) / d[k]
                Jdy += (self.anchor_point[anchor_index[k]][ANCHORy] - (resulty - momentumSize * vdy)) * (d[k] - anchor_dist[k]) / d[k]
                sum_cost += d[k]
            vdx = momentumSize * vdx - stepSize * Jdx
            vdy = momentumSize * vdy - stepSize * Jdy
            resultx -= vdx
            resulty -= vdy
            if Jdx < accuracy * 5 and Jdx > -1 * accuracy * 5 and \
                    Jdy < accuracy * 5 and Jdy > -1 * accuracy * 5 and (i > 50):
                stepSize = 0.01
                momentumSize = 0.9
            if (((sum_cost < accuracy) or ((Jdx < accuracy and Jdx > -1 * accuracy) \
                                           and (Jdy < accuracy and Jdy > -1 * accuracy))) and (
                    i > 100)):
                break
        resultXList = []
        resultYList = []
        resultXList.append(resultx)
        resultYList.append(resulty)

        # if resultx > boundaryXmin and resultx < boundaryXmax and resulty > boundaryYmin and resulty < boundaryYmax:
        self.list_resultx.extend([resultx])
        self.list_resulty.extend([resulty])
        del self.list_resultx[0]
        del self.list_resulty[0]
        self.avg_resultx = np.clip((sum(self.list_resultx) - max(self.list_resultx) - min(self.list_resultx)) / 3.0,
                              self.boundaryXmin, self.boundaryXmax)
        self.avg_resulty = np.clip((sum(self.list_resulty) - max(self.list_resulty) - min(self.list_resulty)) / 3.0,
                              self.boundaryYmin, self.boundaryYmax)

# ser = serial.Serial(
#     port='/dev/ttyUSB1',
#     baudrate=115200)

# c=CalculatePosition()

# while True:
#     if ser.readable():
#         res = ser.readline()
#         if len(res.decode().split(','))>=6:
#             c.SumValues(res.decode()[1:len(res)-1])
#             print(c.avg_resultx, c.avg_resulty)


# c.SumValues("2,1.17,5,1.39,4,2.33")
# print(c.avg_resultx,c.avg_resulty)
# c.SumValues("5,1.43,2,0.95,3,1.22,4,2.34")
# print(c.avg_resultx,c.avg_resulty)
# c.SumValues("4,2.24,5,1.46,6,1.51,2,1.22")
# print(c.avg_resultx,c.avg_resulty)
# c.SumValues("3,1.23,2,1.25,4,2.29,3,1.21")
# print(c.avg_resultx,c.avg_resulty)
# 6,1.48,4,2.25,5,1.51,3,1.16,6,1.65
# 3,1.18

# 5,1.43,2,0.95,3,1.22,4,2.34
# 4,2.24,5,1.46,6,1.51,2,1.22
# 3,1.23,2,1.25,4,2.29,3,1.21
# 2,1.17,5,1.39,4,2.33
# 2,1.12,3,1.20
# 3,1.24,5,1.33