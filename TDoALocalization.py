import numpy as np;

c = 0.299792458 # m/ns
class TDoALocalization:
    @staticmethod
    def coordinatesInTermsOfR(base_anchor, anchors):
        K = list(map(lambda anchor : anchor.X_POS*anchor.X_POS + anchor.Y_POS*anchor.Y_POS, anchors))
        base_K = base_anchor.X_POS*base_anchor.X_POS + base_anchor.Y_POS*base_anchor.Y_POS
        anchor_coordinate_differences = list(map(lambda anchor : (anchor.X_POS - base_anchor.X_POS, anchor.Y_POS - base_anchor.Y_POS), anchors))
        R = list(map(lambda anchor : anchor.TDOA_BASE * c, anchors))

        coordinate_difference_matrix = -1 * np.linalg.inv(np.array([[anchor_coordinate_differences[0][0], anchor_coordinate_differences[0][1]], [anchor_coordinate_differences[1][0], anchor_coordinate_differences[1][1]]]))
        range_difference_matrix = np.array([[R[0]], [R[1]]])
        range_k_matrix = 0.5 * np.array([[R[0] - K[0] + base_K], [R[1] - K[1] + base_K]])

        r_coefficient = (np.matmul(coordinate_difference_matrix, range_difference_matrix))
        constant = (np.matmul(coordinate_difference_matrix, range_k_matrix))

        print("Coefficient of R: " + str(r_coefficient))
        print("Constant added: " + str(constant))
        
        

