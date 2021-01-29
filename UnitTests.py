from Anchor import anchor
import math
import numpy as np
from TDoALocalization import TDoALocalization
import numpy as np

class UnitTest:
    """
        ms = (x, y)
        bts_i = (x_i, y_i)

        In Chan-Ho approximation, bts1 will be treated as the base anchor
        In Harbi approximation, bts1 and bts2 will be treated as base anchors
        
    """
    def __init__(self, ms, bts1, bts2, bts3, unitTestID):
        self.mobileStation = ms

        self.c = 0.299792458 # m/ns

        r1 = self.getDistanceToMS(bts1[0], bts1[1])
        anchor1 = anchor(1, float(bts1[0]), float(bts1[1]), 0.0)

        r2 = self.getDistanceToMS(bts2[0], bts2[1])
        anchor2 = anchor(1, float(bts2[0]), float(bts2[1]), (float(r2) - float(r1)) / self.c)

        r3 = self.getDistanceToMS(bts3[0], bts3[1])
        anchor3 = anchor(2, float(bts3[0]), float(bts3[1]), (float(r3) - float(r1)) / self.c)

        self.anchors = (anchor1, anchor2, anchor3)

        self.unitTestID = unitTestID

        self.noise = np.array([0, 0, 0])

        print(f"UNIT TEST {self.unitTestID} BEGINNING=============================================================\n")
    
    def printDetails(self):
        print(f"Noise: {self.noise}\n")
        print(f"Mobile station: ({self.mobileStation[0]}, {self.mobileStation[1]})")
        print(f"Anchor1: ({self.anchors[0].X_POS}, {self.anchors[0].Y_POS})\tNoisy TDOA: {self.anchors[0].TDOA_BASE}\tNoiseless TDOA: {self.anchors[0].TDOA_BASE - self.noise[0]}")
        print(f"Anchor2: ({self.anchors[1].X_POS}, {self.anchors[1].Y_POS})\tNoisy TDOA: {self.anchors[1].TDOA_BASE}\tNoiseless TDOA: {self.anchors[1].TDOA_BASE - self.noise[1]}")
        print(f"Anchor3: ({self.anchors[2].X_POS}, {self.anchors[2].Y_POS})\tNoisy TDOA: {self.anchors[2].TDOA_BASE}\tNoiseless TDOA: {self.anchors[2].TDOA_BASE - self.noise[2]}")
        
    def addNoise(self):
        # Add AWGN (additive white gaussian noise) to the TDOAs
        # to introduce delay estimation error as described in Chan Ho
        # paper
        
        # 0.001 / c^2 is the noise power given in the Chan paper,
        # and in white Gaussian noise the variance (whose square
        # # root is the standard deviation) is is a measure
        # of the noise power
        self.noise = np.random.normal(0, math.sqrt(0.001 / (self.c**2)), 3)

        self.anchors[0].TDOA_BASE += self.noise[0]
        self.anchors[1].TDOA_BASE += self.noise[1]
        self.anchors[2].TDOA_BASE += self.noise[2]

    def getDistanceToMS(self, x, y):
        return math.sqrt((x - self.mobileStation[0]) ** 2 + (y - self.mobileStation[1]) ** 2)

    def getChanHoApproximation(self, verbose=False):

        print(f"\nCHAN HO APPROXIMATION: {self.unitTestID}-----------------------------\n")

        localization_algorithms = TDoALocalization()

        BASE_ANCHOR = self.anchors[0]
        ANCHOR_1 = self.anchors[1]
        ANCHOR_2 = self.anchors[2]

        r_coefficient, constant = localization_algorithms.coordinatesInTermsOfR(BASE_ANCHOR, (ANCHOR_1, ANCHOR_2))

        if verbose:
            print(f"R Coefficient:\n{r_coefficient}\n")
            print(f"Constant:\n{constant}\n")

        chanho_approximated_r = localization_algorithms.chanHoApproximationOfR(BASE_ANCHOR)

        if verbose:
            print(f"Chan Ho R:\n{chanho_approximated_r}\n")

        if(chanho_approximated_r > 0):
            approx_x = r_coefficient[0][0]*chanho_approximated_r + constant[0][0]
            approx_y = r_coefficient[1][0]*chanho_approximated_r + constant[1][0]

            print(f"Actual x: {self.mobileStation[0]}\tChan-Ho x: {approx_x}\tError: {round((approx_x - self.mobileStation[0]) / self.mobileStation[0] * 100, 3)}%")
            print(f"Actual y: {self.mobileStation[1]}\tChan-Ho y: {approx_y}\tError: {round((approx_y - self.mobileStation[1]) / self.mobileStation[1] * 100, 3)}%\n")

            return np.array([approx_x, approx_y]), np.array([self.mobileStation[0], self.mobileStation[1]])

        else:
            raise RuntimeError("TDoA values out of range")

    def getHarbiApproximation(self, verbose=False):

        print(f"\nHARBI APPROXIMATION: {self.unitTestID}-------------------------------\n")

        localization_algorithms = TDoALocalization()

        anchorA = self.anchors[0]
        anchorB = self.anchors[1]
        anchorC = self.anchors[2]
                
        x, y = localization_algorithms.harbiApproximation((anchorA, anchorB, anchorC))
        
        # convert from 1 x 1 np arrays to regular numbers
        x = x[0]
        y = y[0]

        print(f"Actual x: {self.mobileStation[0]}\tHarbi X: {x}\tError: {round((x - self.mobileStation[0]) / self.mobileStation[0] * 100, 3)}%")
        print(f"Actual y: {self.mobileStation[1]}\tHarbi Y: {y}\tError: {round((y - self.mobileStation[1]) / self.mobileStation[1] * 100, 3)}%\n")

        return np.array([x, y]), np.array([self.mobileStation[0], self.mobileStation[1]])

    def finish(self):
        print(f"==============================================================UNIT TEST {self.unitTestID} FINISHED\n")