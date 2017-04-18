''' This module is for handling playground rules. Playgrounds can load these rules into effect. Once
loaded into a playground, these rules determine the behavoiur of various weaselballs playing within
the playground. Nodes connected to weaselballs (electromagnets, physical connectors, etc.) may or not be affected
by playground rules. This file is the parent for specific rules.'''

'''
list of stuff to get to

go through the motion of ball
figure out how exporting data and information

'''

import numpy as np

class Rules(object):
    '''Creates the object that holds all the rules'''
    def __init__(self, **keyword_list):
        self.speedLimit = .3
        self.listOfMethods = [] #Can hold the list of methods for this particular rule set
        self.acceleration = 0 #acceleration factor
        self.largest_valid_rotation = 30
        self.turn_length = 5# milliseconds
        self.enableRangedEffects = True
        self.riskTurnDelay = 10
        self.riskThreshold = self.speedLimit * 9 #totally random without thought. Could be important
        self.preferred_conflict_method = False #True if rules dictact a special way of contact handling
        self.ballSize = .25
        for item in keyword_list:
            if item == 'speedLimit' and (isinstance(item, float) or isinstance(item, int)):
                self.speedLimit = keyword_list[item]
    def ball_acceleration(self, ball):
        '''This methods handles the rules for how acceleration
        is determined in this playground. A 0 for rule acceleration
        indicates that the balls all travel at speed limit'''
        if self.acceleration != 0:
            pass #handle acceleration physics, I need to do this
        else:
            ball.speed = self.speedLimit

    def ball_rotation(self, ball):
        '''This function will generate a weight rotation value
        that can be used to generate the new direction the ball
        is facing. It currently uses a standardized normal distribution'''
        random_val = np.random.standard_normal()
        rotation = random_val * self.largest_valid_rotation
        ball.orientation += rotation
        if ball.orientation > 360:
            ball.orientation -= 360
        elif ball.orientation < 0:
            ball.orientation += 360

    def direct_contact(self,ballPair):
        for node in ballPair[0].node_types:
            if node.isMagnetic:
                ball1magnetic = True
        for node in ballPair[1].node_types:
            if node.isMagnetic:
                ball2magnetic = True
        if ball2magnetic and ball1magnetic:
            return 'connect'
        else:
            return 'deflect'

    def contact_wall(self, wall, ball):
        original_orientation = ball.orientation
        if wall=='right':
            ball.orientation = -original_orientation
        elif wall=='left':
            ball.orientation = -original_orientation
        elif (wall == 'top'):
            ball.orientation = 180 - original_orientation 
        elif wall == 'bottom':
            ball.orientation = 180 - original_orientation
        self.ball_rotation(ball)
        if ball.orientation > 360:
            ball.orientation -= 360
        elif ball.orientation < 0:
            ball.orientation += 360
        
    def end_of_round(self):
        pass #by default, rules don't have end of round stuff
    