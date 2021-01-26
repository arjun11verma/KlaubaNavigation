from Anchor import anchor;
from TDoALocalization import TDoALocalization;
import math

localization_algorithms = TDoALocalization()

# Arjun test code
BASE_ANCHOR = anchor('base', 0, 0, 0)
ANCHOR_1 = anchor(1, 5, 0, 2)
ANCHOR_2 = anchor(2, 0, 5, 4)

r_coefficient, constant = localization_algorithms.coordinatesInTermsOfR(BASE_ANCHOR, (ANCHOR_1, ANCHOR_2))

print(f'Coefficient of R:\n{r_coefficient}')
print(f'Constant added:\n{constant}')

r_one, r_two = localization_algorithms.chanHoApproximationOfR(BASE_ANCHOR)

print(f'Chan Ho Approximations of R: {r_one}, {r_two}')

chanho_approximated_r = r_one if r_one > 0 else r_two

x, y = localization_algorithms.harbiApproximation((BASE_ANCHOR, ANCHOR_1, ANCHOR_2))

if(chanho_approximated_r > 0):
    print(f'Chan Ho Approximation of (x, y): ({round(r_coefficient[0][0]*chanho_approximated_r + constant[0][0], 3)}, {round(r_coefficient[1][0]*chanho_approximated_r + constant[1][0], 3)})')
    print(f'Harbi Approximation of (x, y): ({round(x[0], 3)}, {round(y[0], 3)})')
else:
    print("TDoA values out of range")


"""
# Mihir test code
anchorA = anchor(1, 1, 2, 0)
anchorB = anchor(2, 3, 1, 1)
anchorC = anchor(3, 0, 0, math.sqrt(2) - 1)

r_coefficient, constant = localization_algorithms.coordinatesInTermsOfR(anchorB, (anchorA, anchorC))

print(f'Coefficient of R:\n{r_coefficient}')
print(f'Constant added:\n{constant}')

r_one, r_two = localization_algorithms.chanHoApproximationOfR(anchorB)

print(f'Chan Ho Approximations of R: {r_one}, {r_two}')

chanho_approximated_r = r_one if r_one > 0 else r_two

if(chanho_approximated_r > 0):
    print(f'Chan Ho Approximation of (x, y): ({round(r_coefficient[0][0]*chanho_approximated_r + constant[0][0], 3)}, {round(r_coefficient[1][0]*chanho_approximated_r + constant[1][0], 3)})')
else:
    print("TDoA values out of range")
"""

# Harvey = Mihir
# ChanHo = Arjun