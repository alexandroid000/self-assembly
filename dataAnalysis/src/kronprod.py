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

class KronProd:
    def __init__(self, As, x):
        self.As = As
        self.flat_A = np.concatenate([a.flatten() for a in self.As], axis=None)
        self.nmat = len(self.As)
        self.n = [len(a) for a in self.As] # dimensions of factors
        self.N = reduce(mul, self.n, 1) # size of final vector y = A*x
        self.Y = np.empty(shape=self.N, dtype = np.float64)
        self.X = x
        #self.Y = np.empty(shape=self.N, dtype = object)
        #self.X = x

    def contract(self, nk, mk, ki):
        ktemp = 0
        inic = ki*(nk*nk)
        for i in range(nk): # dim of matrix k
            J = 0
            for s in range(int(mk)): # N / nk
                I = inic
                #sum = ""
                sum = 0.0
                for t in range(nk): # dim of matrix k
                    #sum = sum +"+"+ self.flat_A[I]+"*("+self.X[J]+")"
                    sum = sum + self.flat_A[I]*self.X[J]
                    if DEBUG:
                        print("elem",I,"of",self.flat_A)
                        print("elem",J,"of",self.X)
                        print("sum=", sum)
                    I = I + 1
                    J = J + 1
                self.Y[ktemp] = sum
                if DEBUG:
                    print("setting element",ktemp,"of Y")
                    print("Y is now", self.Y)
                ktemp = ktemp + 1
            inic = I
            if DEBUG:
                print("inic = ", inic)
        np.copyto(self.X, self.Y)

    def tensorProd(self):
        k = self.nmat
        nk = self.n[k-1]
        mk = self.N/nk
        for ki in range(k):
            if DEBUG:
                print("IN CONTRACTION ",ki)
                print("mk: ", mk)
            mk = self.N/self.n[k-1-ki]
            self.contract(nk, mk, ki)
