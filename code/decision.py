import numpy as np
import math

def dist(x, y):
    return np.sqrt(np.sum((x - y)**2))

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            stop = False
            # Check the extent of navigable terrain
            if Rover.near_sample:
                # If near a sample, then stop to collect it
                stop = True
            elif len(Rover.vel_hist) >= Rover.hist_size_set \
                    and max(Rover.vel_hist) <= 0.2:
                # If rover does not proceed, stop to avoid the obstacle
                stop = True
            elif len(Rover.nav_angles) >= Rover.stop_forward:
                # If navigable terrain looks good
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                nav_angles_right = Rover.nav_angles[Rover.nav_angles < 0]
                # Stick to the right wall whenever is possible
                if len(nav_angles_right) > Rover.go_forward:
                    Rover.steer = np.clip(np.mean(nav_angles_right * 180/np.pi), -15, 15)
                else:
                    # Set steering to average angle with come off set
                    angle = np.mean(Rover.nav_angles * 180/np.pi) + 15
                    Rover.steer = np.clip(angle, -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                stop = True

            if stop:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'

            # Update velocity history
            if Rover.mode != 'forward':
                Rover.vel_hist = []
            else:
                Rover.vel_hist.insert(0, Rover.vel)
                Rover.vel_hist = Rover.vel_hist[:Rover.hist_size_set]

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're stopped, then turn to make some way
            else:
                Rover.steer = Rover.steer_set
                Rover.yaw_start = Rover.yaw
                Rover.mode = 'turn'

        elif Rover.mode == 'turn':
            # Now we're stopped and we have vision data to see if there's a path forward
            if Rover.near_sample:
                Rover.steer = Rover.steer_set
            elif math.fabs(Rover.yaw - Rover.yaw_start) < 30.0:
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                Rover.steer = Rover.steer_set # Could be more clever here about which way to turn
            # If we're stopped but see sufficient navigable terrain in front then go!
            else:
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.yaw_start = Rover.yaw
                else:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    angle_offset = 15
                    angle = np.mean(Rover.nav_angles * 180/np.pi)
                    Rover.steer = np.clip(angle - angle_offset, -15, 15)
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

