'''
PolyominoGenerator.py (for use with Python 3)

To run, use "python PolyominoGenerator.py n" where n is replaced
with the integer of the expected number of units in the polyomino.

This program uses a lexicographic ordering to ensure the same 
configuration is only printed once (though symmetries and transformations
of configurations do appear). The program uses a recursive function that 
adds positive integers as "used" units, and uses negative integers to
enumerate locations for future units to be placed. Any negative integers
with magnitudes greater than the largest positive integer are changed to
-1 to signify that this location cannot be used for future units, though
in the textbook, these numbers are circled instead (this is the basis for 
the lexicographic ordering).

For example:
Following the example from pg. 231 of the geometry textbook used to build
the program, let's begin with a matrix like this:

| 0 | 0 | 0 | 0 | 0 | 0 |
| 0 | 0 | 0 | 0 | 0 | 0 |
| 0 | 0 |-3 | 0 | 0 | 0 |
| 0 | 0 | 1 |-2 | 0 | 0 |
| 0 | 0 | 0 | 0 | 0 | 0 |

The positive "1" represents the only unit currently in the configuration.
The "-2" and "-3" represent possible future placements. Here are the next level
configurations:

| 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |
| 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 |-6 | 0 | 0 | 0 |
| 0 | 0 |-3 |-5 | 0 | 0 |    | 0 |-4 | 3 |-5 | 0 | 0 |
| 0 | 0 | 1 | 2 |-4 | 0 |    | 0 | 0 | 1 |-1 | 0 | 0 |
| 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |

In the left matrix, the unit of "-2" was inverted to add it to the configuration,
and then two more potential placements were added for the next branch. In the right,
"3" was added to the configuration instead, but since this meant the largest number
of the configuration is larger than the magnitude of "-2", the "-2" location was
changed to "-1" to signify it cannot be a placement for future units (as this case is
taken care of with further branches of the left configuration). The daughter branches
of the left matrix are:

| 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |
| 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |
| 0 | 0 |-7 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 |-7 | 0 | 0 |
| 0 |-6 | 3 |-5 | 0 | 0 |    | 0 | 0 |-1 |-5 |-7 | 0 |    | 0 | 0 |-1 | 5 |-6 | 0 |
| 0 | 0 | 1 | 2 |-4 | 0 |    | 0 | 0 | 1 | 2 | 4 |-6 |    | 0 | 0 | 1 | 2 |-1 | 0 |
| 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |

The daughter branches of the right matrix are:

| 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |
| 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 |-9 | 0 | 0 | 0 |
| 0 |-8 |-6 | 0 | 0 | 0 |    | 0 | 0 |-6 |-8 | 0 | 0 |    | 0 |-7 | 6 |-8 | 0 | 0 |
|-7 | 4 | 3 |-5 | 0 | 0 |    | 0 |-1 | 3 | 5 |-7 | 0 |    | 0 |-1 | 3 |-1 | 0 | 0 |
| 0 | 0 | 1 |-1 | 0 | 0 |    | 0 | 0 | 1 |-1 | 0 | 0 |    | 0 | 0 | 1 |-1 | 0 | 0 |
| 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |    | 0 | 0 | 0 | 0 | 0 | 0 |


TODO:
- Apply to 6- and 8-sided shapes as well.
- Deal with different shapes with same perimeter path (ex. 9-unit 3x3 cube vs. 8-unit "hollow" 3x3)

'''

# Importing modules
from __future__ import print_function
import sys
import numpy as np

# Declare Polyomino class
class Polyomino:

    # Constructor function (copy constructor when a copy is used)
    # num_blocks - Number of blocks in final configuration
    # copy - Polyomino used as copy for self
    def __init__(self, num_blocks, copy=None):

        if copy is None:
            self.num_blocks = num_blocks    # Number of blocks in the final configuration
            self.maxInt = 1                 # Largest integer in the configuration
            self.numInts = 1                # Total number of active or "used" integers
            self.size = 1                   # Number of blocks currently placed
            self.xdim = 2*num_blocks-1      # Matrix dimensions
            self.ydim = num_blocks
            self.labelDict = {}             # Dictionary of labels     

            # Clear matrix array, place first in the center of the top row, and
            # pad spaces to the left of the first block with "-1" (they aren't used)
            self.array = np.zeros((self.ydim, self.xdim), dtype=np.int16)
            for n in range(self.num_blocks-1):
                self.array[0][n] = -1
            self.array[0][num_blocks-1] = 1

        else:
            self.num_blocks = copy.num_blocks
            self.maxInt = copy.maxInt
            self.numInts = copy.numInts
            self.size = copy.size
            self.xdim = copy.xdim
            self.ydim = copy.ydim
            self.labelDict = copy.labelDict

            # Initalize matrix array and use copy
            self.array = np.zeros((self.ydim, self.xdim), dtype=np.int16)
            for m in range(self.ydim):
                for n in range(self.xdim):
                    self.array[m][n] = copy.array[m][n]

    # refreshSpaces() runs through the matrix and does the following:
    #   - replaces zeroes next to positive numbers with negative integers
    #   - for negative numbers whose magnitude is greater than the largest
    #       number in the current arrangement, replaces with -1
    def refreshSpaces(self):
        negInt = -self.numInts-1
        for m in range(self.ydim):
            for n in range(self.xdim):
                if self.array[m][n] > 0:
                    if n > 0:
                        if self.array[m][n-1] == 0:
                            self.array[m][n-1] = negInt
                            negInt -= 1
                            self.numInts += 1
                    if n < self.xdim-1:
                        if self.array[m][n+1] == 0:
                            self.array[m][n+1] = negInt
                            negInt -= 1
                            self.numInts += 1
                    if m > 0:
                        if self.array[m-1][n] == 0:
                            self.array[m-1][n] = negInt
                            negInt -= 1
                            self.numInts += 1
                    if m < self.ydim-1:
                        if self.array[m+1][n] == 0:
                            self.array[m+1][n] = negInt
                            negInt -= 1
                            self.numInts += 1
                elif self.array[m][n] < -1:
                    if self.maxInt > -self.array[m][n]:
                        self.array[m][n] = -1

    # recFinder() is a recursive function to find unique arrangements
    def recFinder(self):

        # If final configuration is reached, add label to dictionary and print it
        if self.size == self.num_blocks:
            thisLabel = self.lexLabelOrdering()
            
            # Add label to dict if not currently added
            if thisLabel not in self.labelDict:
                #Check the L/R switched version
                labelList = list(str(thisLabel))
                newLabelList = labelList[::-1]

                labelVal = int(''.join(newLabelList))
                for i in range(len(newLabelList)):
                    newLabelList.insert(0, newLabelList.pop())
                    if int(''.join(newLabelList)) > labelVal:
                        labelVal = int(''.join(newLabelList))

                if thisLabel >= labelVal:
                    self.labelDict[thisLabel] = 1
                    self.printX(thisLabel)

        # If incomplete, test out each potential option to place a new block
        else:
            optionsList = []
            for m in range(self.ydim):
                for n in range(self.xdim):
                    if self.array[m][n] < -1:
                        optionsList.append([m,n])
            for option in range(len(optionsList)):
                newArray = Polyomino(self.num_blocks, self)
                (x,y) = optionsList[option]

                # Invert negative integer to include it in the configuration
                newArray.array[x][y] *= -1
                newArray.size += 1

                # Set new largest integer in configuration as needed
                if newArray.array[x][y] > newArray.maxInt:
                    newArray.maxInt = newArray.array[x][y]
                newArray.refreshSpaces()
                newArray.recFinder()

    # printX() prints a minimalist version of each configuration
    def printX(self, label):
        for m in range(self.ydim):
            for n in range(self.xdim):
                print('X' if self.array[m][n] > 0 else ' ', end='')
            print('')
        print('')
        print('Label: ' + str(label))
        print('')

    # lexLabelOrdering() takes the self.array and returns the proper trinary label
    # for a given configuration with the lowest lexicographic ordering. Starting from
    # the upperleft corner point of the "1" block, starting by facing the right direction, 
    # the label is as follows:
    # 0 - straight
    # 1 - right
    # 2 - left   
    # The lexicographic label is found after the clockwise path is "traversed".
    def lexLabelOrdering(self):
        label = '1'
        direction = 'E'
        pathx = self.num_blocks
        pathy = 0

        # While loop to create label
        while not (pathx == self.num_blocks - 1 and pathy == 0):
            if direction == 'E':
                if pathx < 2*self.num_blocks - 1:
                    if pathy > 0:
                        if self.array[pathy-1][pathx] > 0:
                            # Go left
                            label += '2'
                            direction = 'N'
                            pathy -= 1
                            continue
                    if self.array[pathy][pathx] > 0:
                        # Go straight
                        label += '0'
                        pathx += 1
                        continue
                # Else, go right
                label += '1'
                direction = 'S'
                pathy += 1

            elif direction == 'S':
                if pathy < self.num_blocks:
                    if pathx < 2*self.num_blocks - 1:
                        if self.array[pathy][pathx] > 0:
                            # Go left
                            label += '2'
                            direction = 'E'
                            pathx += 1
                            continue
                    if self.array[pathy][pathx-1] > 0:
                        # Go straight
                        label += '0'
                        pathy += 1
                        continue
                # Else, go right
                label += '1'
                direction = 'W'
                pathx -= 1

            elif direction == 'W':
                if pathx > 0:
                    if pathy < self.num_blocks:
                        if self.array[pathy][pathx-1] > 0:
                            # Go left
                            label += '2'
                            direction = 'S'
                            pathy += 1
                            continue
                    if self.array[pathy-1][pathx-1] > 0:
                        # Go straight
                        label += '0'
                        pathx -= 1
                        continue
                # Else, go right
                label += '1'
                direction = 'N'
                pathy -= 1

            else: #direction == 'N'
                if pathy > 0:
                    if pathx > 0:
                        if self.array[pathy-1][pathx-1] > 0:
                            # Go left
                            label += '2'
                            direction = 'W'
                            pathx -= 1
                            continue
                    if self.array[pathy-1][pathx] > 0:
                        # Go straight
                        label += '0'
                        pathy -= 1
                        continue
                # Else, go right
                label += '1'
                direction = 'E'
                pathx += 1

        # Iteratively shift the label values to 
        # find the true lexicographic label
        labelVal = int(label)
        labelChars = list(label)
        for i in range(len(labelChars)):
            labelChars.insert(0, labelChars.pop())
            if int(''.join(labelChars)) > labelVal:
                labelVal = int(''.join(labelChars))
                    
        return labelVal
        
# Main function body (sys.argv[1] is an integer n in the executable "py PolyominoGenerator.py n")
if __name__ == "__main__":
    polynum = int(sys.argv[1])
    poly = Polyomino(polynum)
    poly.refreshSpaces()
    poly.recFinder()
    print(poly.labelDict.keys())


