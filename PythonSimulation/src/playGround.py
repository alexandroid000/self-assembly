import numpy as np
import weasel as weasel
import rules as rl
class PlayGround(object):
    """Creates the environment for weasalball objects to
    play around in"""
    def __init__(self, rules, **keyword_list):
        #keyword_list stuff:
            #dict with bounds
            #player list with weasalball object
        DEFAULTPLAYSIZE = 3 #default playground height and width, temp
        self.xbound = DEFAULTPLAYSIZE
        self.ybound = DEFAULTPLAYSIZE
        self.players = []
        self.turn_count = 0
        self.ballsAtRisk = [] #holds pairs (list len 2) of balls close enough to
                              #check for conflict
        self.rules = rules
        self.countSinceRiskTest = 0
        self.largestRangedEffect = 0
        for item in keyword_list: ### scrapes for various inputs parameters
            if item == 'players' and isinstance(item, list): #scrape players
                for player in item:
                    self.players.append(player)
            if item == 'playSize': #scrape bounds
                self.xbound = item['xbound']
                self.ybound = item['ybound']

    def player_in_bound(self, ball): ### checks if a particular is in the playground, True
        "Tests in a ball is still within the valid playground"
        return bool(ball.xloc < self.xbound and ball.yloc < self.ybound and ball.xloc > 0 and ball.yloc > 0)

    def add_player(self, new_player):
        "Adds a new player to the list of tracked players in the playground"
        self.players.append(new_player)

    def _check_for_conflicts(self):
        if self.rules.preferred_conflict_method:
            self.rules.conflict_method()
        else:
            for ballPair in self.ballsAtRisk:
                distance = self.distanceBetween(ballPair[0], ballPair[1])
                if  distance < self.rules.ballSize:
                    self.direct_contact(ballPair)
                else:
                    pass #don't bother calculating anything, beyond interference

    def redefineRiskBalls(self):
        self.ballsAtRisk = [] #clears risk list
        for i in range(0,len(self.players)):
            for j in range(i,len(self.players)):
                if self.distanceBetween(self.players[i], self.players[j]) < self.rules.riskThreshold:
                    if self.players[i] != self.players[j]:
                        self.ballsAtRisk.append([self.players[i], self.players[j]])
    def direct_contact(self, ballPair):
        print self.ballsAtRisk
        print self.players
        index = self.players.index(ballPair[1])
        self.players[index] = ballPair[0]        
    def distanceBetween(self, ball1, ball2):
        xpos1 = ball1.xloc
        xpos2 = ball2.xloc
        ypos1 = ball1.yloc
        ypos2 = ball2.yloc
        distance = np.sqrt((xpos2-xpos1)**2 + (ypos2 - ypos1)**2)
        return distance

    def moveBall(self, ball, inputTime):
        '''This method is responsible for managing the movement of
        one particular ball. This does not modify velocity or orientation, only 
        it's location. Balls orientation can move when this method is called
        because upon wall contact, it calls methods that can change it'''
        time = inputTime/1000.
        change_in_distance = time * (ball.velocity)
        change_in_horizontal = change_in_distance * np.sin(np.deg2rad(ball.orientation)) #trig and stuff
        change_in_vertical = change_in_distance * np.cos(np.deg2rad(ball.orientation)) # more trig and stuff
        ball.xloc += change_in_horizontal
        ball.yloc += change_in_vertical #adding our trig stuff to our ball's location
        if not self.player_in_bound(ball):
            if ball.xloc > self.xbound:
                ball.xloc = self.xbound
                self.rules.contact_wall('right',ball)
            if ball.yloc > self.ybound:
                ball.yloc = self.ybound
                self.rules.contact_wall('top', ball)                
            if ball.xloc < 0:
                ball.xloc = 0
                self.rules.contact_wall('left', ball)                
            if ball.yloc < 0 :
                ball.yloc = 0
                self.rules.contact_wall('bottom', ball)             
                   



    def execture_turn(self, **keyword_list):
        '''Handles at the playground level executing
        a turn for every player currently on the roster
        For non-roster players, you need to execute their actions
        explictly'''
        for ball in self.players: ## add optional paramters and stuff
            self.rules.ball_acceleration(ball)
            self.moveBall(ball, self.rules.turn_length) 
        if len(self.players) > 1:
            if self.countSinceRiskTest > self.rules.riskTurnDelay:
                self.redefineRiskBalls()
            else:
                self.countSinceRiskTest = self.countSinceRiskTest + 1
            self._check_for_conflicts() ## playground level function for conflict resolution
        self.rules.end_of_round() #checks if rules ditact anything for end of round checking