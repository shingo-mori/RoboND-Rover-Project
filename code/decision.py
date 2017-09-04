import numpy as np
import math

def dist(x, y):
    return np.sqrt(np.sum((x - y)**2))

def pi(deg):
    return deg * 180 / np.pi

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    if Rover.nav_angles is not None:
        # Do nothing While picking a sample
        if Rover.picking_up:
            pass
        elif Rover.mode == 'forward':
            # Set brake and throttle
            Rover.brake = 0
            if Rover.vel < Rover.max_vel:
                Rover.throttle = Rover.throttle_set
            else:
                Rover.throttle = 0

            # If rover does not proceed, stop to avoid the obstacle
            if len(Rover.vel_hist) >= Rover.hist_size_set \
                    and max(Rover.vel_hist) <= 0.2:
                Rover.mode = 'stop'
            # If there's enough sample pixels, then set direction to the sample
            elif len(Rover.sample_angles) >= Rover.collect_sample\
                    and np.mean(pi(Rover.sample_angles) < 10):
                Rover.steer = np.clip(np.mean(pi(Rover.sample_angles)), -15, 15)
                # If rover is near the sample, then stop to collect it
                if Rover.near_sample:
                    Rover.mode = 'stop'
            # If there's enough navigable terrain pixels then keep going forward
            elif len(Rover.nav_angles) >= Rover.stop_forward:
                # Manage to stick to the right wall
                nav_angles_right = Rover.nav_angles[Rover.nav_angles < 0]
                if len(nav_angles_right) > int(Rover.go_forward):
                    angle = np.mean(pi(nav_angles_right))
                    Rover.steer = np.clip(angle, -15, 15)
                else:
                    angle = np.mean(pi(Rover.nav_angles)) - 15
                    Rover.steer = np.clip(angle, -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                Rover.mode = 'stop'

            # Update velocity history
            if Rover.mode == 'forward':
                Rover.vel_hist.insert(0, Rover.vel)
                Rover.vel_hist = Rover.vel_hist[:Rover.hist_size_set]
            else:
                Rover.vel_hist = []

        elif Rover.mode == 'stop':
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            # If we're still moving or near the sample, then keep braking
            if Rover.near_sample or Rover.vel >= 0.2:
                pass
            # If  we're stopped, then turn to make some way
            elif Rover.vel < 0.2:
                Rover.yaw_start = Rover.yaw
                Rover.mode = 'turn'

        elif Rover.mode == 'turn':
            Rover.throttle = 0
            Rover.brake = 0
            # If we haven't turned enough, then keep turning
            if math.fabs(Rover.yaw - Rover.yaw_start) < 30.0:
                Rover.steer = Rover.steer_set
            else:
                # If we don't see sufficient navigable terrain, then start turning again
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.yaw_start = Rover.yaw
                # If we see sufficient navigable terrain in front then go!
                else:
                    Rover.mode = 'forward'

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

