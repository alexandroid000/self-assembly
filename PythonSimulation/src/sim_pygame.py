import sys, random, math, pygame
from pygame.locals import *
from math import sqrt, cos, sin, atan2, pi
from time import sleep
import playGround as pg
import weasel
import rules
import turtle
XDIM = 500
YDIM = 500
WINSIZE = [XDIM, YDIM]
def PolyToWindowScale(poly):
    newpoly = []
    for p in poly:
        newpoly.append((p[0]/2.0,YDIM - p[1]/2.0))
    return newpoly

def PointToWindowScale(p):
     return (p[0]/2.0,YDIM - p[1]/2.0)

groundRules = rules.Rules()
ball1 = weasel.WeaselBall('ball1', {'xloc' : .5, 'yloc' : .5}, velocity=.25)
ball2 = weasel.WeaselBall('ball2', {'xloc' : .25, 'yloc': .25}, velocity=.25, orientation = 90)
playPlace = pg.PlayGround(groundRules)

playPlace.add_player(ball1)
playPlace.add_player(ball2)
black = 20,20,40
white=255,240,200
green = 50,130,50
screen = pygame.display.set_mode(WINSIZE)
screen.fill(white)
origin = (0.0,0.0)

boundary = (playPlace.xbound, playPlace.ybound)
pygame.draw.polygon(screen, black, PolyToWindowScale(boundary),5)