import numpy as np
from Anchor import anchor
import math
from TDoALocalization import TDoALocalization


if __name__ == "__main__":
    localizer = TDoALocalization()

    a1 = anchor(1, 1, 2, 0)
    a2 = anchor(2, 3, 1, 1)
    a3 = anchor(3, 0, 0, math.sqrt(2) - 1)

    x, y = localizer.harbiApproximation((a1, a2, a3))
    print(f"(x, y) = {x}, {y}")