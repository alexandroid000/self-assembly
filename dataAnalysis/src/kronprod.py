#! /usr/bin/env python
# Code implementing "Efficient Computer Manipulation of Tensor Products..."
# Pereyra Scherer
# Assumes all factor matrices square, identical size
# TODO use pycontracts to enforce this ^

import numpy as np
from operator import mul
from functools import reduce

DEBUG = False

# TODO investigate LinearOperators for even moar fast
#from scipy.sparse.linalg import LinearOperator

def unwrap_mat(A):
    return [elem for row in A for elem in row]

class KronProd:
    def __init__(self, As, x):
        self.As = As
        self.flat_A = np.concatenate([unwrap_mat(a) for a in self.As], axis=None)
        self.nmat = len(self.As)
        self.n = [len(a) for a in self.As] # dimensions of factors
        self.N = reduce(mul, self.n, 1) # size of final vector y = A*x
        self.Y = [0.0]*self.N
        self.X = x

    def contract(self, nk, mk):
        ktemp = 0
        inic = 0
        for i in range(1,nk+1):
            J = 0
            for s in range(1, int(mk+1)):
                I = inic
                sum = 0
                for t in range(1, nk+1):
                    if DEBUG:
                        print("elem",I,"of",self.flat_A)
                        print("elem",J,"of",self.X)
                    sum = sum + self.flat_A[I]*self.X[J]
                    I = I + 1
                    J = J+1
                self.Y[ktemp] = sum
                ktemp = ktemp+1
            inic = I
        for i in range(0,self.N):
            self.X[i] = self.Y[i]


    def tensorProd(self):
        k = self.nmat-1
        nk = self.n[k]
        mk = self.N/nk
        while k >= 0:
            if DEBUG:
                print("IN CONTRACTION ",self.nmat - k)
                print("mk: ", mk)
            k = k-1
            mk = self.N/self.n[k]
            self.contract(nk, mk)

def test(n,p):
    #r_As = [np.random.rand(p,p) for i in range(n)]
    #As = [m/m.sum(axis=1)[:,None] for m in r_As] # normalize each row
    As = [np.array([[1, 1], [1,1]]),
          np.array([[1,1], [1,1]])]
    #x = np.random.rand(p*n)
    x = np.array([1,1,1,1])

    print("efficient values")
    # must reverse list for now
    kp = KronProd(list(reversed(As)), x)
    kp.tensorProd()
    print(kp.Y)

    print("correct values")
    big_A = reduce(np.kron, As)
    x = np.array([1,1,1,1])
    print("A = ", big_A)
    print("x = ", x)
    big_y = np.matmul(big_A, x)
    print("y = ", big_y)

