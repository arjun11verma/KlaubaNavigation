import numpy as np;

c = 0.299792458 # m/ns
class TDoALocalization:
    @staticmethod
    def coordinatesInTermsOfR(base_anchor, anchors):
        K = list(map(lambda anchor : anchor.X_POS*anchor.X_POS + anchor.Y_POS*anchor.Y_POS, anchors))
        anchor_coordinate_differences = list(map(lambda anchor : (anchor.X_POS - base_anchor.X_POS, anchor.Y_POS - base_anchor.Y_POS), anchors))
        R = list(map(lambda anchor : anchor.TDOA_BASE * c, anchors))
        

