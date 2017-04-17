 '''def _set_velocity(self, acceleration, turn_time):
        This internal function handles accleration
        and can check to make sure nothing is exceeding its
        internal speed limit. When it is called, it will directly
        modify the objects velocity list
        acceleration_x_direction = turn_time * acceleration * np.cos(np.radians(self.orientation))
        acceleration_y_direction = turn_time * acceleration * np.sin(np.radians(self.orientation))
        self.velocity[0] = self.velocity[0] + acceleration_x_direction
        self.velocity[1] = self.velocity[1] + acceleration_y_direction'''
        '''currently being moved to rules set'''
'Pulled from weasel.py during restructuring of rules and playgrounds'



    def take_turn(self, **keyword_list):
        '''This method handles a standard turn
        It rotates for us, then moves us based on optional
        turn length size (fraction of second it should sim)
        Wall contact is handled by playground'''
        DEFAULT_TURN_LENGTH = .01
        turn_time = DEFAULT_TURN_LENGTH
        acceleration = 1.
        self._set_rotation() #rotate
        if 'turn_time' in keyword_list: ###MOVE TO PLAYGROUND
            turn_time = keyword_list['turn_time']
        if 'acceleration' in keyword_list:
            acceleration = keyword_list['acceleration']
        self._set_velocity(acceleration, turn_time)
        self.xloc += self.velocity[0] * turn_time
        self.yloc += self.velocity[1] * turn_time



if (not self.player_in_bound(ball)): ## icky if statements, avoid. T'is a silly place
            #figure out what wall we need to work with
            if change_in_horizontal + ball.xloc < 0:
                if change_in_vertical + ball.yloc < 0 or change_in_vertical + ball.yloc > self.ybound:
                    if change_in_vertical > change_in_horizontal: ## we will hit top/bottom first
                        if change_in_vertical + ball.yloc < 0: ## down under
                            impactWall = 'bottom'
                        else:
                            impactWall = 'top'
                else:
                    impactWall = 'right'
            elif change_in_horizontal + ball.xloc > self.xbound:
                if change_in_vertical + ball.yloc < 0 or change_in_vertical + ball.yloc > self.ybound:
                    if change_in_vertical > change_in_horizontal: ## we will hit top/bottom first
                        if change_in_vertical + ball.yloc < 0: ## down under
                            impactWall = 'bottom'
                        else:
                            impactWall = 'top'
                else:
                    impactWall = 'left'
            elif change_in_vertical + ball.yloc > self.ybound:
                impactWall = 'top'
            else:
                impactWall = 'bottom'
        else:
            impactWall = 'None'
        #we know which wall we impact first. Now it is time to recursively call moveBall
        #to move us the remainder of the distance
        impactWall
        if impactWall == 'None':
            pass
        elif impactWall == 'top':
            distance_until_impact = self.ybound - ball.yloc
            if ball.velocity != 0:
                time_to_impact = distance_until_impact / ball.velocity
                time_left = time - time_to_impact
                self.rules.contact_wall('top', ball)
 

        elif impactWall == 'bottom':
            distance_until_impact = ball.yloc
            if ball.velocity != 0:
                time_to_impact = distance_until_impact / ball.velocity
                time_left = time - time_to_impact
                self.rules.contact_wall('bottom', ball)
                
                

        elif impactWall == 'right':
            distance_until_impact = ball.xloc
            if ball.velocity != 0:
                time_to_impact = distance_until_impact / ball.velocity
                time_left = time - time_to_impact
                self.rules.contact_wall('right', ball)
                
                

        elif impactWall == 'left':
            distance_until_impact = self.xbound - ball.xloc
            if ball.velocity != 0:
                time_to_impact = distance_until_impact / ball.velocity
                time_left = time - time_to_impact
                self.rules.contact_wall('left', ball)   




'''
assembly object for holding collisions and bounded balls



'''