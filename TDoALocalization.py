import numpy as np;
import math;

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
        
        range_squared_coefficient = 1 - self.r_coefficient[0][0]*self.r_coefficient[0][0] - self.r_coefficient[1][0]*self.r_coefficient[1][0]
        range_coefficient = 2*base_anchor.X_POS*self.r_coefficient[0][0] + 2*base_anchor.Y_POS*self.r_coefficient[1][0] - 2*self.r_coefficient[0][0]*self.constant[0][0] - 4*self.r_coefficient[1][0]*self.constant[1][0]
        range_constant = -1*(self.base_K - 2*base_anchor.X_POS*self.constant[0][0] - 2*base_anchor.Y_POS*self.constant[1][0] + self.constant[0][0]*self.constant[0][0] + self.constant[1][0]*self.constant[1][0])

        try:
            r_discriminant = math.sqrt(range_coefficient ** 2 - 4*range_squared_coefficient*range_constant)
        except ValueError:
            raise ValueError("These values result in an imaginary R")

        return (-1*range_coefficient + r_discriminant)/(2*range_squared_coefficient), (-1*range_coefficient - r_discriminant)/(2*range_squared_coefficient)
        
    """
    anchors = list of three anchors
    """
    def harbiApproximation(self, anchors):
        base_K = anchors[0].X_POS ** 2 + anchors[0].Y_POS ** 2
        K = self.getK((anchors[1], anchors[2]))
        
        # home BTS = anchors[0]
        
        r_coeff_0, constant_0 = self.coordinatesInTermsOfR(anchors[0], (anchors[1], anchors[2]))
        r_one_0, r_two_0 = self.chanHoApproximationOfR(anchors[0])

        chanHoR1 = r_one_0 if r_one_0 > 0 else r_two_0
        xo1 = r_coeff_0[0][0] * chanHoR1 + constant_0[0][0]
        yo1 = r_coeff_0[1][0] * chanHoR1 + constant_0[1][0]
        
        # home BTS = anchors[1]

        r_coeff_1, constant_1 = self.coordinatesInTermsOfR(anchors[1], (anchors[0], anchors[2]))
        r_one_1, r_two_1 = self.chanHoApproximationOfR(anchors[1])
        
        chanHoR2 = r_one_1 if r_one_1 > 0 else r_two_1
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

        x, y = np.matmul(coordinate_difference_matrix, range_difference_matrix * omega + range_k_matrix)

        return x, y