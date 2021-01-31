from UnitTests import UnitTest
from random import randint
from matplotlib import pyplot
import traceback
import numpy as np
import math

print()

"""

Mobile station: (3, 6)
Anchor 1: (4.0, 5.0)
Anchor 2: (9.0, 8.0)
Anchor 3: (8.0, 7.0)
"""
"""
ms = (3, 6)
bts1 = (4, 5)
bts2 = (9, 8)
bts3 = (8, 7)

test = UnitTest(ms, bts1, bts2, bts3, 1)
test.printDetails()
test.getChanHoApproximation()
test.getHarbiApproximation()
test.finish()

"""

"""
ms = (7, 10)
bts1 = (3, 9)
bts2 = (4, 8)
bts3 = (10, 3)

test = UnitTest(ms, bts1, bts2, bts3, 1)
test.addNoise()
test.printDetails()
test.getChanHoApproximation()
test.getHarbiApproximation()
test.finish()
"""

lower_limit = 0
upper_limit = 10

def generate_range():
    return (randint(lower_limit, upper_limit), randint(lower_limit, upper_limit))

TOTAL_TRIALS = 1

chanho_coordinate_errors = []
harbi_coordinate_errors = []
chanho_mobile_station_positions = []
harbi_mobile_station_positions = []

for i in range(TOTAL_TRIALS):
    ms   = (3, 4)

    bts1 = (lower_limit, lower_limit)
    bts2 = (lower_limit, upper_limit)
    bts3 = (upper_limit, lower_limit)

    """
    bts1 = (generate_range())
    while bts1 == ms:
        bts1 = (generate_range())

    bts2 = (generate_range())
    while bts2 == bts1 or bts2 == ms:
        bts2 = (generate_range())

    bts3 = (generate_range())
    while bts3 == bts2 or bts3 == bts1 or bts3 == ms:
        bts3 = (generate_range())
    """

    test = UnitTest(ms, bts1, bts2, bts3, i)
    #test.addNoise()
    test.printDetails()

    try:
        chanho_coordinate_errors.append(test.getChanHoApproximation(verbose=True))
        chanho_mobile_station_positions.append(math.atan2(ms[1], ms[0]) * 180/math.pi)
    except Exception as ex:
        print(f"Test failed:")
        traceback.print_exc()
        print()

    try:
        harbi_coordinate_errors.append(test.getHarbiApproximation())
        harbi_mobile_station_positions.append(math.atan2(ms[1], ms[0]) * 180/math.pi)
    except Exception as ex:
        print(f"Test failed:")
        traceback.print_exc()
        print()
    
    test.finish()

"""

average_chanho_error = 0
average_harbi_error = 0

chanho_errors = []
harbi_errors = []

def vector_magnitude(position_vector):
    return math.sqrt(sum(x ** 2 for x in position_vector))

def relative_vector_error(coordinate_errors, vector_errors):
    average_error = 0

    for error in coordinate_errors:
        approximated_vector = error[0]
        real_vector = error[1]

        if(math.isnan(approximated_vector[0]) or math.isnan(approximated_vector[1])): 
            vector_errors.append(0)
            pass
        
        difference_vector = np.subtract(approximated_vector, real_vector)
        average_error += abs(vector_magnitude(difference_vector)/vector_magnitude(approximated_vector))
        vector_errors.append(abs(vector_magnitude(difference_vector)/vector_magnitude(approximated_vector)))
    
    return average_error/TOTAL_TRIALS

average_harbi_error = relative_vector_error(harbi_coordinate_errors, harbi_errors)

average_chanho_error = relative_vector_error(chanho_coordinate_errors, chanho_errors)

print(f'Average relative vector error of ChanHo Approximation over {TOTAL_TRIALS} trials: {round(average_chanho_error * 100, 3)}%')

print(f'Average relative vector error of Harbi Approximation over {TOTAL_TRIALS} trials: {round(average_harbi_error * 100, 3)}%')

def generatePlot(x_data, y_data, x_title, y_title, title):
    pyplot.step(x_data, y_data, 'o', color = 'black')
    pyplot.title(title)
    pyplot.xlabel(x_title)
    pyplot.ylabel(y_title)
    pyplot.show()

generatePlot(np.array(chanho_mobile_station_positions), [round(error * 100, 3) for error in chanho_errors], "Mobile Station Angle from x axis (degrees)", "Relative Vector Percent Error", "Chanho estimation errors by Mobile Station Position" )

"""

