from UnitTests import UnitTest
from random import randint
import traceback

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


for i in range(1, 11):

    # Generate random positions for the mobile station and
    # anchors, while making sure not to place multiple things
    # at the same location

    ms   = (randint(1, 10), randint(1, 10))

    bts1 = (randint(1, 10), randint(1, 10))
    while bts1 == ms:
        bts1 = (randint(1, 10), randint(1, 10))

    bts2 = (randint(1, 10), randint(1, 10))
    while bts2 == bts1 or bts2 == ms:
        bts2 = (randint(1, 10), randint(1, 10))

    bts3 = (randint(1, 10), randint(1, 10))
    while bts3 == bts2 or bts3 == bts1 or bts3 == ms:
        bts3 = (randint(1, 10), randint(1, 10))

    test = UnitTest(ms, bts1, bts2, bts3, i)
    test.addNoise()
    test.printDetails()

    try:
        test.getChanHoApproximation()
    except Exception as ex:
        print(f"Test failed:")
        traceback.print_exc()
        print()

    try:
        test.getHarbiApproximation()
    except Exception as ex:
        print(f"Test failed:")
        traceback.print_exc()
        print()
    
    test.finish()
