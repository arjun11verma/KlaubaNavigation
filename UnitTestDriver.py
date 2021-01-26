from UnitTests import UnitTest
from random import randint
import traceback

for i in range(1, 11):
    ms   = (randint(1, 10), randint(1, 10))
    bts1 = (randint(1, 10), randint(1, 10))
    bts2 = (randint(1, 10), randint(1, 10))
    bts3 = (randint(1, 10), randint(1, 10))


    test = UnitTest(ms, bts1, bts2, bts3, i)
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