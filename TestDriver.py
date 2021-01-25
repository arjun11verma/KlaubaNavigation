from Anchor import anchor;
from TDoALocalization import TDoALocalization;

BASE_ANCHOR = anchor('base', 0, 0, 0)
ANCHOR_1 = anchor(1, 5, 0, 10)
ANCHOR_2 = anchor(2, 0, 5, 2)

localization_algorithms = TDoALocalization()

r_coefficient, constant = localization_algorithms.coordinatesInTermsOfR(BASE_ANCHOR, (ANCHOR_1, ANCHOR_2))

print("Coefficient of R: " + str(r_coefficient))
print("Constant added: " + str(constant))

r_one, r_two = localization_algorithms.chanHoApproximationOfR(BASE_ANCHOR)

print("Approximations of R: " + str(r_one) + ", " + str(r_two))

chanho_approximated_r = r_one if r_one > 0 else r_two

if(chanho_approximated_r > 0):
    print(f'Chan Ho Approximation of (x, y): {round(r_coefficient[0][0]*chanho_approximated_r + constant[0][0], 3)}, {round(r_coefficient[1][0]*chanho_approximated_r + constant[1][0], 3)}')
else:
    print("TDoA values out of range")

# Harvey = Mihir
# ChanHo = Arjun