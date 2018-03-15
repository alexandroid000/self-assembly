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

    
#Function takes in 2-D coordinate array and returns list of available spaces around it
    # For each coordinate in shape
        # add each surrounding coordinate to list
        # Check for existing coordinates
            # Do not add multiples
    # Add all potential spaces for each existing coordinate,
        # Remove existing coordinates

max_x = 10
max_y = 12
coor_array

available_spaces():
    out_spaces = [['0' for x in range(max_x)] for y in range(max_y)]
    for i in range(0, x_max):
        for j in range(0, y_max):
            if coor_array[i[j]] == 1:
                
                out_spaces[i[j]] = 1


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
'''
def functionName(arg1, arg2, arg3):
    return