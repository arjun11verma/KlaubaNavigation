import numpy as np;
from Anchor import anchor;

BASE_ANCHOR = anchor(1, 0, 0, 0)
ANCHOR_1 = anchor(2, 50, 0, 20)
ANCHOR_2 = anchor(3, 0, 50, 10)
c = 0.299792458 # m/ns

def coordinatesInTermsOfR(base_anchor, anchors):
    K = list(map(lambda anchor : anchor.X_POS*anchor.X_POS + anchor.Y_POS*anchor.Y_POS, anchors))
    anchor_coordinate_differences = list(map(lambda anchor : (anchor.X_POS - base_anchor.X_POS, anchor.Y_POS - base_anchor.Y_POS), anchors))
    R = list(map(lambda anchor : anchor.TDOA_BASE * c, anchors))

coordinatesInTermsOfR(BASE_ANCHOR, (ANCHOR_1, ANCHOR_2))
    

# Distance from coordinates to anchor i : R(i) = sqrt((x(1) - x)^2 + ((y(1) - y))^2)
# TDoA * c = R(i) - R(1)

