#! /usr/bin/env python

import unittest

from src.kronprod import *

class TestKron(unittest.TestCase):

    # add global stuff here
    def setUp(self):
        return

#    def testOnes(self):
#        A1 = [ np.array([[1, 1], [1,1]]),
#                    np.array([[1,1], [1,1]])]
#        x1 = np.array([1,1,1,1])
#        y1 = np.array([4,4,4,4])
#        kp = KronProd(list(reversed(A1)), x1)
#        kp.tensorProd()
#        self.assertSequenceEqual(list(kp.Y), list(y1))

#    def testString(self):
#        A1 = [ np.array([["a11", "a12"], ["a21","a22"]]),
#                    np.array([["b11","b12"], ["b21","b22"]])]
#        x1 = np.array(["x1","x2","x3","x4"], dtype=object)
#        big_A = reduce(np.kron, A1)
#        print(big_A)
#        print(x1)
#        big_y = np.matmul(big_A, x1)
#        print("full calc: ",big_y)
#        kp = KronProd(list(reversed(A1)), x1)
#        kp.tensorProd()
#        print("efficient calc: ", kp.Y)
#        self.assertSequenceEqual(list(kp.Y), list(big_y))

    def testInts(self):
        A1 = [ np.array([[1.0, 0.0], [0.0,0.0]]),
                    np.array([[1.,1.], [0.,0.]])]
        x1 = np.array([1.,2.,3.,4.])
        big_A = reduce(np.kron, A1)
        print(big_A)
        print(x1)
        big_y = np.matmul(big_A, x1)
        print("full calc: ",big_y)
        kp = KronProd(list(reversed(A1)), x1)
        kp.tensorProd()
        print("efficient calc: ", kp.Y)
        self.assertSequenceEqual(list(kp.Y), list(big_y))

    # this dimensionality pushes the limit of what full rank calc can do
    def testRandom(self):
        n = 5 # number of factors
        p = 7 # dimension of factor
        r_As = [np.random.rand(p,p) for i in range(n)]
        As = [m/m.sum(axis=1)[:,None] for m in r_As] # normalize each row
        x = np.random.rand(p**n)

        big_A = reduce(np.kron, As)
        big_y = np.matmul(big_A, x)
        print("full calc: ",big_y)

        kp = KronProd(list(reversed(As)), x)
        kp.tensorProd()
        print("efficient calc: ", kp.Y)

        np.testing.assert_almost_equal(big_y, kp.Y, decimal=7, verbose=True)

    # took ~150 seconds
    def testBig(self):
        n = 5 # number of factors
        p = 20 # dimension of factor
        r_As = [np.random.rand(p,p) for i in range(n)]
        As = [m/m.sum(axis=1)[:,None] for m in r_As] # normalize each row
        x = np.random.rand(p**n)
        kp = KronProd(list(reversed(As)), x)
        kp.tensorProd()
        print("efficient calc: ", kp.Y)


if __name__ == '__main__':
    unittest.main()
