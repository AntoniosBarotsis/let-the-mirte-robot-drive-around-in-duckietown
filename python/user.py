import time

from lane_follower import Follower

follower = Follower()

# Just loop the input for testing purposes
while True:
    print(follower.get_lane())
    # get user input, if it's "q" then quit
    inp = input("q? ")
    if inp == "q":
        break
