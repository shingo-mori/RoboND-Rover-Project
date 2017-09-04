## Project: Search and Sample Return

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/binary_images.png
[image2]: ./misc/image_conversion.png
[image3]: ./misc/drive_image.png

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

I modified the `color_thresh()` function to allow not only higher color threshold, but also lower color threshold.
I used the following thresholds to identify terrain, rock, and obstacle pixels.

- Terrain: 160 < R < 255, 160 < G < 255, 160 < B < 255
- Rock: 100 < R < 255, 100 < G < 255, 0 < B < 70
- Obstacle: 0 < R < 140, 0 < G < 140, 0 < B < 140

Bellow are the examples of how the applied thresholds work.

![alt text][image1]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

First, top-down view image is created from camera image using `perspective_transform()` function.
I set the source and destination points to map each pixel of the top-down view to 0.1x0.1 meter square on the ground.
Then binary images are created using the same color thresholds as mentioned above.
Those binary images are converted to rover-centric x and y positions using the `rover_coords()` function.
Finally, rover-centric positions are mapped to world coordinates using the `pix_to_world()` function.
I set the `scale` to 10 since the world map has a resolution of 1 square meter per pixel.

Bellow is the example of image conversion process.

![alt text][image2]

The video output of my test data is in output/ folder.

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

In the `perception_step()` function, updating the world map is implemented almost the same way as the above `process_image()` function, but additionally row and roll/pitch thresholds are added to prevent from updating with unreliable image sets. 
I added some code to update Rover pixel distances and angles for terrain and rock.

The main changes I made on `decision_step()` are as follows:

- avoid getting stuck on obstacles
  - To detect that Rover is getting stuck on obstacle, velocity history is added to `RoverState` class. If the velocity of  Rover keeps being low for some amount of frames, it means Rover is having some trouble going forward (because of the obstacle).
  - To avoid the getting-stuck situation, `turn` mode is added to start and keep turning until Rover finds some way to go forward.
- collect samples
  - If there are enough pixels of a sample in the right side (>10 deg), `Rover.steer` is set to the mean angle of sample pixels.
  - Rover stops when `Rover.near_sample` flag is set to `True`. It stops until the sample is collected.
- try to stick on the right walls
  - If there are enough terrain pixels on the right side, `Rover.steer` is set to the mean angle of right-terrain pixels.
  - Angle offset (-15) is applied when steering angle is set.
  - The above two methods keeps Rover running along the right walls.
  
#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

##### Simulator
| Resolution | Graphics quality | FPS   |
| :---:      | :---:            | :---: |
| 1024x768   | Good             | 30~40 |

##### Result
| Time   | Mapped | Fidelity | Located | Collected |
| :---:  | :---:  | :---:    | :---:   | :---:     |
| 622.0s | 97.6%  | 80.4%    | 6       | 6         |

 The below images are screenshots of the simulator during autonomous navigation.
 
![alt text][image3]
 
##### To improve the result

- Implement function to return to the starting point.
- Allow higher maximum velocity in open areas to optimize the time.
- Implement "visited world map" to detect unvisited places and to prevent from moving into already visited places.
- Prevent Rover from going in a infinite circle. This can be achieved by checking if the steering angle stays the same for certain amount of frames.


