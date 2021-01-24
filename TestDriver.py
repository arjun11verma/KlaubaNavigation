from Anchor import anchor;
from TDoALocalization import TDoALocalization;

BASE_ANCHOR = anchor(1, 0, 0, 0)
ANCHOR_1 = anchor(2, 50, 0, 20)
ANCHOR_2 = anchor(3, 0, 50, 10)

TDoALocalization.coordinatesInTermsOfR(BASE_ANCHOR, (ANCHOR_1, ANCHOR_2))
