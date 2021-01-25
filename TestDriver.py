from Anchor import anchor;
from TDoALocalization import TDoALocalization;

BASE_ANCHOR = anchor('base', 0, 0, 0)
ANCHOR_1 = anchor(1, 5, 0, 20)
ANCHOR_2 = anchor(2, 0, 5, 10)

localization_algorithms = TDoALocalization()

r_coefficient, constant = localization_algorithms.coordinatesInTermsOfR(BASE_ANCHOR, (ANCHOR_1, ANCHOR_2))

print("Coefficient of R: " + str(r_coefficient))
print("Constant added: " + str(constant))

r_one, r_two = localization_algorithms.chanHoApproximationOfR(BASE_ANCHOR)

chanho_approximated_r = r_one if r_coefficient[0][0]*r_one + constant[0][0] > 0 and r_coefficient[1][0]*r_one + constant[1][0] > 0 else r_two

print("Chan Ho Approximation of (x, y): " + str(r_coefficient[0][0]*chanho_approximated_r + constant[0][0]) + ", " +  str(r_coefficient[1][0]*chanho_approximated_r + constant[1][0]))

# Harvey = Mihir
# ChanHo = Arjun