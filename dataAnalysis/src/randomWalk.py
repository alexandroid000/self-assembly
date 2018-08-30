#The purpose of this file is to allow for creation of a random walk matrix that can be used for testing functions by giving an artificial transition matrix
from sklearn.preprocessing import normalize
import numpy as np
import random

class RandomWalk():
    def __init__(self, size):
        self.size = size
        self.M = buildRandomWalk()
    
    def buildRandomWalk(self):
        M = np.zeros( (size,size) )
        #Add artificial data
        for (row in range(size))
        {
            #M goes from 1 to n
            M[row,row] = M[row,row] + 1
            if(row > 0)
            {
                M[row-1, row-1] = M[row-1, row-1] + 1
                M[row-1, row] = M[row-1, row] + 1
                M[row, row-1] = M[row, row-1] + 1
            }
            if(row < n-1)
            {
                M[row+1, row+1] = M[row+1, row+1] + 1
                M[row+1, row] = M[row+1, row] + 1
                M[row, row+1] = M[row, row+1] + 1
            }
        }
        # defining the number of steps
        n = 100000
        x = size/2
        y = size/2
        #Artificially add some 
        # filling the coordinates with random variables
        for i in range(1, n):
            val = random.randint(1, 4)
            if val == 1:
                x += 1
                if(x >= size):
                    x = size-1
                M[x][y] += 1
            elif val == 2:
                x -= 1
                if(x < 0):
                    x = 0
                M[x][y] += 1
            elif val == 3:
                y += 1
                if(y >= size):
                    y = size-1
                M[x][y] += 1
            else:
                y -= 1
                if(y < 0):
                    y=0
                M[x][y] += 1
        normalized_new_matrix = normalize(M, norm='l1', axis=1)
        return normalized_new_matrix