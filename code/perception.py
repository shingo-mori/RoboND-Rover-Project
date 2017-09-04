import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh_low=(160, 160, 160), rgb_thresh_high=(255, 255, 255)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh_low[0]) \
                & (img[:,:,1] > rgb_thresh_low[1]) \
                & (img[:,:,2] > rgb_thresh_low[2]) \
                & (img[:,:,0] < rgb_thresh_high[0]) \
                & (img[:,:,1] < rgb_thresh_high[1]) \
                & (img[:,:,2] < rgb_thresh_high[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Identify pixels above the row threshold
def row_thresh(threshed, row_thresh=0):
    row_select = threshed[:,:]
    row_select[:row_thresh, :] = 0
    return row_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    img = Rover.img
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                      [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                      ])
    # 2) Apply perspective transform
    warped = perspect_transform(img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    terrain_thresh = color_thresh(warped)
    rock_thresh = color_thresh(warped, rgb_thresh_low=(100, 100, 0), rgb_thresh_high=(255, 255, 70))
    obstacle_thresh = color_thresh(warped, rgb_thresh_low=(0, 0, 0), rgb_thresh_high=(140, 140, 140))
    # exclude rock pixels from the obstacle image
    obstacle_thresh[rock_thresh] = 0
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:, :, 0] = obstacle_thresh * 255
    Rover.vision_image[:, :, 1] = rock_thresh * 255
    Rover.vision_image[:, :, 2] = terrain_thresh * 255

    angle_thresh_min = 359.5
    angle_thresh_max = 0.5
    if not (angle_thresh_max < Rover.roll < angle_thresh_min) \
            and not (angle_thresh_max < Rover.pitch < angle_thresh_min):

        # 5) Convert map image pixel values to rover-centric coords
        thresh_row = int(img.shape[0] * 7 / 10)
        terrain_map_thresh = row_thresh(terrain_thresh, thresh_row)
        terrain_xpix, terrain_ypix = rover_coords(terrain_map_thresh)
        rock_map_thresh = row_thresh(rock_thresh, thresh_row)
        rock_xpix, rock_ypix = rover_coords(rock_map_thresh)
        obstacle_map_thresh = row_thresh(obstacle_thresh, thresh_row)
        obstacle_xpix, obstacle_ypix = rover_coords(obstacle_map_thresh)

        # 6) Convert rover-centric pixel values to world coordinates
        scale = 10
        terrain_x_world, terrain_y_world =\
            pix_to_world(terrain_xpix, terrain_ypix, Rover.pos[0],
                         Rover.pos[1], Rover.yaw,
                         Rover.worldmap.shape[0], scale)
        rock_x_world, rock_y_world =\
            pix_to_world(rock_xpix, rock_ypix, Rover.pos[0],
                         Rover.pos[1], Rover.yaw,
                         Rover.worldmap.shape[0], scale)
        obstacle_x_world, obstacle_y_world =\
            pix_to_world(obstacle_xpix, obstacle_ypix, Rover.pos[0],
                         Rover.pos[1], Rover.yaw,
                         Rover.worldmap.shape[0], scale)
        # 7) Update Rover worldmap (to be displayed on right side of screen)
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[terrain_y_world, terrain_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    terrain_xpix, terrain_ypix = rover_coords(terrain_thresh)
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(terrain_xpix, terrain_ypix)

    rock_xpix, rock_ypix = rover_coords(rock_thresh)
    Rover.sample_dists, Rover.sample_angles = to_polar_coords(rock_xpix, rock_ypix)

    return Rover