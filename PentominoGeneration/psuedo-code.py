from __future__ import print_function


'''
Function
input: Number of units n
Output: Visual display of all possible arrangements
    2n-1 array of each potential shape with first unit in middle


#makeShape(n) takes in
makeShape(n):
    if n == 1:
        baseShapes = [empty list]
    baseShapes = makeShapes(n-1)
    # 

'''

    
#Function takes in 2-D coordinate array and returns list of available spaces around it
    # For each coordinate in shape
        # add each surrounding coordinate to list
        # Check for existing coordinates
            # Do not add multiples
    # Add all potential spaces for each existing coordinate,
        # Remove existing coordinates

#Function to flip y

#Function to flip x

#Function that takes in two shapes (coordinate lists) and checks if they are equal

#Function to convert makeShape output into visual representation


'''
functionName(arg1, arg2, arg3)
arg1: integer
arg2: array
arg3:
output:
Description: function takes in a number of arguments equal to arg1
def functionName(arg1, arg2, arg3):
    return
'''




max_x = 10
max_y = 6
coor_array = [['0' for x in range(max_x)] for y in range(max_y)]
out_spaces = [['0' for x in range(max_x)] for y in range(max_y)]

def available_spaces():
    #Generate initial output 2D array
    #out_spaces = [['0' for x in range(max_x)] for y in range(max_y)]
    #Run through each space in the input array
    for i in range(0, max_y):
        for j in range(0, max_x):
            #If element exists at that space, fill outer spaces with dots
            if coor_array[i][j] == 1:
                #Inner dots
                if (i != 0) & (j != 0) & (i < max_x) & (j < max_y):
                    out_spaces[i-1][j] = 1
                    out_spaces[i+1][j] = 1
                    out_spaces[i][j-1] = 1
                    out_spaces[i][j+1] = 1
                #Edge dots (not corners)
                #Top edge
                elif (i != 0) & (j != 0) & (i < max_x):
                    out_spaces[i-1][j] = 1
                    out_spaces[i+1][j] = 1
                    out_spaces[i][j-1] = 1
                #Right edge
                elif (i != 0) & (j != 0) & (j < max_y):
                    out_spaces[i-1][j] = 1
                    out_spaces[i][j-1] = 1
                    out_spaces[i][j+1] = 1
                #Bottom edge
                elif (i != 0) & (i < max_x) & (j < max_y):
                    out_spaces[i-1][j] = 1
                    out_spaces[i+1][j] = 1
                    out_spaces[i][j+1] = 1
                #Left edge
                elif (j != 0) & (i < max_x) & (j < max_y):
                    out_spaces[i+1][j] = 1
                    out_spaces[i][j-1] = 1
                    out_spaces[i][j+1] = 1
                #Corners
                #Bottom left
                elif (i == 0) & (j == 0):
                    out_spaces[i+1][j] = 1
                    out_spaces[i][j+1] = 1
                #Top left
                elif (i == 0) & (j == max_y):
                    out_spaces[i+1][j] = 1
                    out_spaces[i][j-1] = 1
                #Bottom right
                elif (i == max_x) & (j == 0):
                    out_spaces[i-1][j] = 1
                    out_spaces[i][j+1] = 1
                #Top right
                elif (i == max_x) & (j == max_y):
                    out_spaces[i-1][j] = 1
                    out_spaces[i][j-1] = 1
    #Run through each space in the input array
    for i in range(0, max_y):
        for j in range(0, max_x):
            #If element exists at that space, remove from surrounding coors
            if coor_array[i][j] == 1:
                out_spaces[i][j] = 0

def print_array(array):
    # Initialize output string
    out_string = [['.' for x in range(max_x)] for y in range(max_y)]

    # Populate out_string
    for row in range(max_y):
        for elem in range(max_x):
            if array[row][elem] == 1:
                out_string[row][elem] = 'X'

    # Print out array
    for row in range(max_y):
        for elem in range(max_x):
            print(out_string[row][elem], end='')
        print('')

coor_array[3][8]= 1
coor_array[1][0] = 1
available_spaces()
print_array(coor_array)
print('')
print_array(out_spaces)
