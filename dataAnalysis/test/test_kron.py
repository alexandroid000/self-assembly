#! /usr/bin/env python

import unittest

from src.kronprod import *

class TestKron(unittest.TestCase):

    # add global stuff here
    def setUp(self):
        return

    def testOnes(self):
        A1 = [ np.array([[1, 1], [1,1]]),
                    np.array([[1,1], [1,1]])]
        x1 = np.array([1,1,1,1])
        y1 = np.array([4,4,4,4])
        kp = KronProd(list(reversed(A1)), x1)
        kp.tensorProd()
        self.assertSequenceEqual(list(kp.Y), list(y1))


if __name__ == '__main__':
    unittest.main()
