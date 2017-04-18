import turtle # Allows us to use turtles

import random

wn = turtle.Screen() # Creates a playground for turtles

turtlebase = turtle.Turtle() #make a turtle that doesn't move so the user can see how far the turtle have moved

turtle1 = turtle.Turtle() # Create the 3 moving turtles and assign it to it's name

turtle2 = turtle.Turtle()

turtle3 = turtle.Turtle()

turtle1.color("blue") #make the colour of the turtle blue

turtle1.pencolor("blue") #make it's line colour blue

turtle2.color("Green")

turtle2.pencolor("green")

turtle3.color("red")

turtle3.pencolor("red")

count = 0 #make a variable called count

while count <= 2000: #make a counter loop

    turtle1.setheading(random.randint(0,360))# choose an angle between 0 and 360. This tells the heading of the turtle

    turtle1.forward(random.randint(-10,10)) #choose a random integer between -10 and 10. This tell how far the turtle is going to move

    turtle2.setheading(random.randint(0,360))

    turtle2.forward(random.randint(-15,15))

    turtle3.setheading(random.randint(0,360))

    turtle3.forward(random.randint(-20,20))

    count = count +1 #increment the counter to move onto the next section of the loop

   # wn.mainloop() # Wait for user to close window
