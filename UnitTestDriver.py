from UnitTests import UnitTest
from random import randint

for i in range(1, 10):
    ms   = (randint(1, 10), randint(1, 10))
    bts1 = (randint(1, 10), randint(1, 10))
    bts2 = (randint(1, 10), randint(1, 10))
    bts3 = (randint(1, 10), randint(1, 10))

    try:
        test = UnitTest(ms, bts1, bts2, bts3, i)
        test.printDetails()
        test.getChanHoApproximation()
        test.getHarbiApproximation()
        test.finish()
    except (ValueError, RuntimeError):
        continue