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

    
#Function takes in 2-D coordinate array and returns list of available spaces
    # around it
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