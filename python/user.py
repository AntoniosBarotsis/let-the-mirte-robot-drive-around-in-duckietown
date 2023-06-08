from lane_follower import Follower

follower = Follower()

# Just loop the input for testing purposes
while True:
    # get user input, if it's "q" then quit
    inp = input("action: ")
    if inp == "q":
        follower.stop_following()
        break
    if inp == "start":
        follower.start_following()
    if inp == "stop":
        follower.stop_following()
