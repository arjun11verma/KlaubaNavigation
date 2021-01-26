from UnitTests import UnitTest
from random import randint
import traceback
import numpy as np
import math

print()

"""
ms = (7, 8)
bts1 = (8, 7)
bts2 = (6, 1)
bts3 = (9, 3)

test = UnitTest(ms, bts1, bts2, bts3, 1)
test.printDetails()
test.getChanHoApproximation()
test.getHarbiApproximation()
test.finish()
"""

TOTAL_TRIALS = 50

chanho_coordinate_errors = []
harbi_coordinate_errors = []

for i in range(TOTAL_TRIALS):
    ms   = (randint(1, 10), randint(1, 10))
    bts1 = (randint(1, 10), randint(1, 10))
    bts2 = (randint(1, 10), randint(1, 10))
    bts3 = (randint(1, 10), randint(1, 10))


    test = UnitTest(ms, bts1, bts2, bts3, i)
    test.printDetails()

    try:
        chanho_coordinate_errors.append(test.getChanHoApproximation())
    except Exception as ex:
        print(f"Test failed:")
        traceback.print_exc()
        print()

    try:
        harbi_coordinate_errors.append(test.getHarbiApproximation())
    except Exception as ex:
        print(f"Test failed:")
        traceback.print_exc()
        print()
    
    test.finish()

average_chanho_error = 0
average_harbi_error = 0

def vector_magnitude(position_vector):
    return math.sqrt(sum(x ** 2 for x in position_vector))

for error in chanho_coordinate_errors:
    approximated_vector = error[0]
    real_vector = error[1]
    difference_vector = np.subtract(approximated_vector, real_vector)
    average_chanho_error += abs(vector_magnitude(difference_vector)/vector_magnitude(approximated_vector))

for error in harbi_coordinate_errors:
    approximated_vector = error[0]
    real_vector = error[1]
    difference_vector = np.subtract(approximated_vector, real_vector)
    average_harbi_error += abs(vector_magnitude(difference_vector)/vector_magnitude(approximated_vector))

average_chanho_error /= TOTAL_TRIALS
average_harbi_error /= TOTAL_TRIALS

print(f'Average vector error of ChanHo Approximation over {TOTAL_TRIALS} trials: {round(average_chanho_error * 100, 3)}%')

print(f'Average vector error of Harbi Approximation over {TOTAL_TRIALS} trials: {round(average_harbi_error * 100, 3)}%')

