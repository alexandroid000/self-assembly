import sys, random, math, pygame
from pygame.locals import *
from math import sqrt, cos, sin, atan2, pi
from time import sleep
import playGround as pg
import weasel
import rules
import turtle

groundRules = rules.Rules()
ball1 = weasel.WeaselBall('ball1', {'xloc' : 2.25, 'yloc' : .4}, velocity=.25, orientation = 27)
ball2 = weasel.WeaselBall('ball2', {'xloc' : .25, 'yloc': .25}, velocity=.25, orientation = 105)
ball3 = weasel.WeaselBall('ball1', {'xloc' : 1.25, 'yloc' : 2.4}, velocity=.25, orientation = 290)

playPlace = pg.PlayGround(groundRules)
print playPlace.xbound
print playPlace.ybound
playPlace.add_player(ball1)
playPlace.add_player(ball2)
playPlace.add_player(ball3)


wn = turtle.Screen()
turtle.setworldcoordinates(0,0,playPlace.xbound, playPlace.ybound)
turtle1 = turtle.Turtle() # Create the 3 moving turtles and assign it to it's name
turtle1.color("blue") #make the colour of the turtle blue
turtle1.pencolor("blue") #make it's line colour blue
turtle1.penup()
turtle1.setx(ball1.xloc)
turtle1.sety(ball1.yloc)
turtle1.pendown()

turtle2 = turtle.Turtle()
turtle2.color("Green")
turtle2.pencolor("green")
turtle2.penup()
turtle2.setx(ball2.xloc)
turtle2.sety(ball2.yloc)
turtle2.pendown()

turtle3 = turtle.Turtle()
turtle3.color("red")
turtle3.pencolor("red")
turtle3.penup()
turtle3.setx(ball3.xloc)
turtle3.sety(ball3.yloc)
turtle3.pendown()

count = 0 #make a variable called count

while count < 500:
    for i in range(50):
        playPlace.execture_turn()
    turtle1.goto(playPlace.players[0].xloc,playPlace.players[0].yloc)
    turtle1.setheading(playPlace.players[0].orientation+90)
    turtle2.goto(playPlace.players[1].xloc,playPlace.players[1].yloc)
    turtle2.setheading(playPlace.players[1].orientation+90)
    turtle3.goto(playPlace.players[2].xloc,playPlace.players[2].yloc)
    turtle3.setheading(playPlace.players[2].orientation+90)
    print count
    print ball1.dump_stats()
    print ball2.dump_stats()
    print playPlace.players
    count = count + 1
turtle.done()

        