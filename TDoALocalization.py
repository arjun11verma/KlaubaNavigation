import numpy as np;
import math;

c = 0.299792458 # m/ns
class TDoALocalization:
    def coordinatesInTermsOfR(self, base_anchor, anchors):
        self.base_K = base_anchor.X_POS*base_anchor.X_POS + base_anchor.Y_POS*base_anchor.Y_POS
        K = list(map(lambda anchor : anchor.X_POS*anchor.X_POS + anchor.Y_POS*anchor.Y_POS, anchors))
        anchor_coordinate_differences = list(map(lambda anchor : (anchor.X_POS - base_anchor.X_POS, anchor.Y_POS - base_anchor.Y_POS), anchors))
        R = list(map(lambda anchor : anchor.TDOA_BASE * c, anchors))

        coordinate_difference_matrix = -1 * np.linalg.inv(np.array([[anchor_coordinate_differences[0][0], anchor_coordinate_differences[0][1]], [anchor_coordinate_differences[1][0], anchor_coordinate_differences[1][1]]]))
        range_difference_matrix = np.array([[R[0]], [R[1]]])
        range_k_matrix = 0.5 * np.array([[R[0]*R[0] - K[0] + self.base_K], [R[1]*R[1] - K[1] + self.base_K]])

        self.r_coefficient = (np.matmul(coordinate_difference_matrix, range_difference_matrix))
        self.constant = (np.matmul(coordinate_difference_matrix, range_k_matrix))

        return self.r_coefficient, self.constant
    
    def chanHoApproximationOfR(self, base_anchor):
        # since r is equal to the distance formula, R(1) = K(1) - 2X(1)x - 2Y(1)y + x^2 + y^2
        if not hasattr(self, 'base_K'):
            self.base_K = base_anchor.X_POS*base_anchor.X_POS + base_anchor.Y_POS*base_anchor.Y_POS
        
        if not (hasattr(self, 'r_coefficient') and hasattr(self, 'constant')):
            raise AttributeError("Please call the coordinates in terms of R method before approximating R")
        
        range_squared_coefficient = 1 - self.r_coefficient[0][0]*self.r_coefficient[0][0] - self.r_coefficient[1][0]*self.r_coefficient[1][0]
        range_coefficient = 2*base_anchor.X_POS*self.r_coefficient[0][0] + 2*base_anchor.Y_POS*self.r_coefficient[1][0] - 2*self.r_coefficient[0][0]*self.constant[0][0] - 2*2*self.r_coefficient[1][0]*self.constant[1][0]
        range_constant = -1*(self.base_K - 2*base_anchor.X_POS*self.constant[0][0] - 2*base_anchor.Y_POS*self.constant[1][0] + self.constant[0][0]*self.constant[0][0] + self.constant[1][0]*self.constant[1][0])

        r_discriminant = math.sqrt(range_coefficient ** 2 - 4*range_squared_coefficient*range_constant)

        return (-1*range_coefficient + r_discriminant)/(2*range_squared_coefficient), (-1*range_coefficient - r_discriminant)/(2*range_squared_coefficient)
        
        

