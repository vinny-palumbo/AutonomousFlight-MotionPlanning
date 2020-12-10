## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

The original `motion_planning.py` script builds up on top of the basic `backyard_flyer_solution.py` script by implementing an A* plath-planning algorithm that executes before the TAKEOFF state. Instead of simply flying in a predetermined box trajectory, it sets an arbitrary goal position on the map (initially 10 units North, 10 units East from the start position) and plans a path to reach that goal without flying into any obstacle on the map. In order to plan a path, there are multiple utility functions from `planning_utils.py` that are used:
- `create_grid()` read a .csv file containing the map data and returns grid representation of a 2D configuration space based on given obstacle data, the drone's altitude and an arbitrary safety distance from the obstacles.
- `valid_actions()` returns a list of valid actions given a configuration space's grid and the current position of the drone. The possible actions are: NORTH, SOUTH, EAST and WEST.
- `a_star()` executes the A* path-planning algorithm to find the least costly trajectory between the start and goal positions (using the Euclidean distance between the drone and the goal implemented in `heuristic()`), given the configuration space and the drone's valid actions at each position throughtout that trajectory.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

To set the global home position (i.e: the goal position), I've read the first line of the `colliders.csv` file, removed the labels and spaces from the header's string; and extracted the latitude and longitude as floating point values. I then set the global home position using the `set_home_position()` function with those values and an altitude of 0.

``` 
# read lat0, lon0 from colliders into floating point values
with open("colliders.csv") as f:
    lat0, lon0 = f.readline().replace('lat0', '').replace('lon0', '').replace(' ', '').split(',')
lat0, lon0 = float(lat0), float(lon0)

# set home position to (lon0, lat0, 0)
self.set_home_position(lon0, lat0, 0)
``` 

#### 2. Set your current local position

I've retrieved the current local position in geodetic coordinates (north, east, down) by using the utility function `global_to_local()` with the parameters: `current_global_position` and `self.global_home`; where `current_global_position` is a numpy array consisting of the current global longitude, latitude and altitude and `self.global_home` is the global home position from the previous step.

``` 
# retrieve current global position
current_global_position = np.array([self._longitude, self._latitude, self._altitude])

# convert to current local position using global_to_local()
current_local_position = global_to_local(current_global_position, self.global_home)
``` 

#### 3. Set grid start position from local position

I set the grid start position with the north and east values of the current local position from the previous step. The grid offsets were substracted so that the coordinates (0,0) now represent the lower-left corner of the grid.

``` 
# convert start position to current position rather than map center
grid_start = (int(current_local_position[0]-north_offset), int(current_local_position[1]-east_offset))
``` 

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

I set the grid's goal position by setting an arbitrary goal global position with a longitude of -122.3964, a latitude of 37.7955 and an altitude of 0. I then converted it in geodetic coordinates by using `global_to_local()`, and substracting the grid's north and east offsets as in the previous step.

``` 
# adapt to set goal as latitude / longitude position and convert
goal_global_position = (-122.3964, 37.7955, 0)
goal_local_position = global_to_local(goal_global_position, self.global_home)
grid_goal = (int(goal_local_position[0]-north_offset), int(goal_local_position[1]-east_offset))
``` 

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

I've modified the A* path-planning algorithm to include diagonal motion by adding these following diagonal actions in the `Action()` class, which have a cost of sqrt(2) instead of 1:
``` 
# add diagonal motions
NORTH_EAST = (-1, 1, np.sqrt(2))
SOUTH_EAST = (1, 1, np.sqrt(2))
SOUTH_WEST = (1, -1, np.sqrt(2))
NORTH_WEST = (-1, -1, np.sqrt(2))
``` 

I've also added some logic to the `valid_actions()` function to remove invalid diagonal actions, depending on the drone's position on the grid:
``` 
# remove invalid diagonal motions
if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
    valid_actions.remove(Action.NORTH_EAST)
if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
    valid_actions.remove(Action.NORTH_WEST)
if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
    valid_actions.remove(Action.SOUTH_EAST)
if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
    valid_actions.remove(Action.SOUTH_WEST)
``` 

#### 6. Cull waypoints 

When a line can be drawn across three subsequent waypoints on a path, the middle waypoint is unnecessary and only the first and last waypoints on that trajectory can be kept to simplify the path, and execute the mission faster. A collinearity test can be used to eliminate those superfluous waypoints and prune the path. This is what the `prune_path()` function does by calculating the determinant of the 3x3 matrix made out of the 3-dimension values of each 3 subsequents waypoints on the path.




