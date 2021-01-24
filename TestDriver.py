from Anchor import anchor;
from TDoALocalization import TDoALocalization;

BASE_ANCHOR = anchor('base', 0, 0, 0)
ANCHOR_1 = anchor(1, 5, 0, 20)
ANCHOR_2 = anchor(2, 0, 5, 10)

TDoALocalization.coordinatesInTermsOfR(BASE_ANCHOR, (ANCHOR_1, ANCHOR_2))
