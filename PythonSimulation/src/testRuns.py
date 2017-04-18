import weasel as weasel

ball1 = weasel.WeaselBall('ball1', {"xloc": 0, 'yloc': 0}, velocity = [1,1]) #create a ball1 object

playGround = weasel.PlayGround()

playGround.add_player(ball1)

playGround.player_in_bound(ball1)
for i in range(0,10):
    playGround.execture_turn()
    stats = ball1.dump_stats()
    print(stats)
    checkBounds = playGround.player_in_bound(ball1)
    print(checkBounds)
