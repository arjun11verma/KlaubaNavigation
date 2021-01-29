import numpy as np;
import math;
from Anchor import anchor

c = 0.299792458 # m/ns

class TDoALocalization:

    def getCoordinateDifferenceMatrix(self, anchor_coordinate_differences):
        return -1 * np.linalg.inv(
            np.array(
                [[anchor_coordinate_differences[0][0], anchor_coordinate_differences[0][1]],
                 [anchor_coordinate_differences[1][0], anchor_coordinate_differences[1][1]]]
            )
        )

    def getAnchorCoordinateDifferences(self, base_anchor, anchors):
        return [(anchor.X_POS - base_anchor.X_POS, anchor.Y_POS - base_anchor.Y_POS) for anchor in anchors]

    def getR(self, anchors):
        return [anchor.TDOA_BASE * c for anchor in anchors]

    def getRangeDifferenceMatrix(self, R):
        return np.array([[R[0]], [R[1]]])

    def getRangeKMatrix(self, R, base_K, K):
        return 0.5 * np.array([[R[0]*R[0] - K[0] + base_K], [R[1]*R[1] - K[1] + base_K]])

    def getK(self, anchors):
        return [anchor.X_POS**2 + anchor.Y_POS**2 for anchor in anchors]

    def coordinatesInTermsOfR(self, base_anchor, anchors):
        self.base_K = base_anchor.X_POS**2 + base_anchor.Y_POS**2
        K = self.getK(anchors)
        anchor_coordinate_differences = self.getAnchorCoordinateDifferences(base_anchor, anchors)
        R = self.getR(anchors)

        coordinate_difference_matrix = self.getCoordinateDifferenceMatrix(anchor_coordinate_differences)
        range_difference_matrix = self.getRangeDifferenceMatrix(R)
        range_k_matrix = self.getRangeKMatrix(R, self.base_K, K)

        self.r_coefficient = (np.matmul(coordinate_difference_matrix, range_difference_matrix))
        self.constant = (np.matmul(coordinate_difference_matrix, range_k_matrix))

        return self.r_coefficient, self.constant
    
    def chanHoApproximationOfR(self, base_anchor):
        # since r is equal to the distance formula, R(1) = K(1) - 2X(1)x - 2Y(1)y + x^2 + y^2
        if not hasattr(self, 'base_K'):
            self.base_K = base_anchor.X_POS**2 + base_anchor.Y_POS**2
        
        if not (hasattr(self, 'r_coefficient') and hasattr(self, 'constant')):
            raise AttributeError("Please call the coordinates in terms of R method before approximating R")
        
        # 0 = (r1_squared_coeff) * (r1)^2 + (r1_linear_coeff) * (r1) + (r1_const)
        # x = a * r1 + b
        # y = c * r1 + d

        a = self.r_coefficient[0][0]
        b = self.constant[0][0]
        c = self.r_coefficient[1][0]
        d = self.constant[1][0]

        x1 = base_anchor.X_POS
        y1 = base_anchor.Y_POS

        k1 = self.base_K

        r1_squared_coeff = a**2 + c**2 - 1
        r1_linear_coeff = -2*x1*a - 2*y1*c + 2*a*b + 2*c*d
        r1_const = k1 - 2*x1*b - 2*y1*d + b**2 + d**2

        # For a quadratic
        # 0 = ax^2 + bx + c
        # we have
        # x = (-b +- sqrt(b^2 - 4ac)) / (2a)

        discriminant_sq = r1_linear_coeff**2 - 4*r1_squared_coeff*r1_const
        if discriminant_sq < 0:
            raise ValueError("These values result in an imaginary R")

        discriminant = math.sqrt(discriminant_sq)
        r1_one = ((-r1_linear_coeff) + discriminant) / (2 * r1_squared_coeff)
        r1_two = ((-r1_linear_coeff) - discriminant) / (2 * r1_squared_coeff)

        return min(r1_one, r1_two) if (r1_one > 0 and r1_two > 0) else max(r1_one, r1_two)

    """
    anchors = list of three anchors
    """
    def harbiApproximation(self, anchors):

        base_K = anchors[0].X_POS ** 2 + anchors[0].Y_POS ** 2
        K = self.getK((anchors[1], anchors[2]))
        
        # home BTS = anchors[0]

        # Create deep copies of each object since we'll be modifying them and don't
        # want to touch the originals
        baseAnchor = anchor('base', anchors[0].X_POS, anchors[0].Y_POS, anchors[0].TDOA_BASE)
        anchor1 = anchor(1, anchors[1].X_POS, anchors[1].Y_POS, anchors[1].TDOA_BASE)
        anchor2 = anchor(2, anchors[2].X_POS, anchors[2].Y_POS, anchors[2].TDOA_BASE)

        r_coeff_0, constant_0 = self.coordinatesInTermsOfR(baseAnchor, (anchor1, anchor2))
        chanHoR1 = self.chanHoApproximationOfR(baseAnchor)

        if(chanHoR1 < 0): 
            raise ValueError("TDoA Values out of Range")
        xo1 = r_coeff_0[0][0] * chanHoR1 + constant_0[0][0]
        yo1 = r_coeff_0[1][0] * chanHoR1 + constant_0[1][0]

        # home BTS = anchors[1]

        # Adjust the TDOAs to make anchors[1] the base anchor from which
        # the other TDOAs are measured

        TDOA_to_subtract = anchors[1].TDOA_BASE

        baseAnchor = anchor('base', anchors[1].X_POS, anchors[1].Y_POS, 0)
        anchor1 = anchor(1, anchors[0].X_POS, anchors[0].Y_POS, anchors[0].TDOA_BASE - TDOA_to_subtract)
        anchor2 = anchor(2, anchors[2].X_POS, anchors[2].Y_POS, anchors[2].TDOA_BASE - TDOA_to_subtract)
        
        r_coeff_1, constant_1 = self.coordinatesInTermsOfR(baseAnchor, (anchor1, anchor2))
        chanHoR2 = self.chanHoApproximationOfR(baseAnchor)
        
        if(chanHoR2 < 0):
            raise ValueError("TDoA Values out of Range")
        xo2 = r_coeff_1[0][0] * chanHoR2 + constant_1[0][0]
        yo2 = r_coeff_1[1][0] * chanHoR2 + constant_1[1][0]

        x1 = anchors[0].X_POS
        y1 = anchors[0].Y_POS
        x2 = anchors[1].X_POS
        y2 = anchors[1].Y_POS
        
        Q = np.array(
            [[ (x1 - xo1) / chanHoR1, (x2 - xo2) / chanHoR2],
             [ (y1 - yo1) / chanHoR1, (y2 - yo2) / chanHoR2]]
        )

        Qt = np.transpose(Q)

        omega = math.sqrt(0.5 * np.trace(np.matmul(Qt, Q)))

        anchor_coordinate_differences = self.getAnchorCoordinateDifferences(anchors[0], (anchors[1], anchors[2]))
        coordinate_difference_matrix = self.getCoordinateDifferenceMatrix(anchor_coordinate_differences)

        R = self.getR((anchors[1], anchors[2]))
        range_difference_matrix = self.getRangeDifferenceMatrix(R)
        range_k_matrix = self.getRangeKMatrix(R, base_K, K)

        x, y = np.matmul(coordinate_difference_matrix, range_difference_matrix * omega * chanHoR1 + range_k_matrix)

        return x, y