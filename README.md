# Path-Planning
This project highlights the implementation of a path planner in C++ that safely drives a vehicle around a highway track with good time efficiency(speed).

Successful driving around for 10 miles without violations!
![Image cropped to region of interest](https://github.com/ashsiv/Path-Planning-/blob/master/output%20images/10%20miles.JPG)

## Project Introduction
The path planner is implemented in main.cpp file under the 'src' directory.

The path planner iterates every 20ms. The input includes

1. **Perception data** (lidar/sensor fusion) on the neighbouring vehicles. Sensor fusion data is a 2d vector of cars and then that car's  unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates etc. 
2. **Highway map** -Each waypoint in the list contains  (x,y,s,dx,dy) values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.
3. **Localization** information

**Step 1** of the path planner involves (using the above information), determine=ing one of the necessary actions

 * keep lane
 * lane change left
 * lane change right
 * increase speed
 * decrease speed
 
**Step 2** involves defining the waypoint trajectory.

### Step 1: Determine Action

Firstly, if the car is in rightmost lane (lane 2) lane changes to the right are avoided. Similarly if the car is in the left most lane (lane 0), lane changes to left are avoided.

Action state variable initialization:
```
// Define action state varibles
          bool too_close = false;
          bool lane_change_left =true;
          bool lane_change_right=true;
          bool changed = false;
```
Next, for every neighbouring car in the lidar sensor's vicinity, the associated frenet coordinates (s,d), speed of the neighbouring car are evaluated to make if lane changes to right or left is safe.

Note: A 40 m safe gap distance ('s') is chosen for cars ahead and a 10 m safe gap distance is chosen for cars behind to decide if lane change to the respective lane (left or right) is safe. A 'd' distance of 6 m is used for picking cars in the vicinity (nighopuring lane).

```
// check if the neighbouring car (ahead or behind) is in proximity. (40m safe gap is chosen for front car and 10m safe gap is chosen                for car behind to do safe lane change)
             if(((check_car_s>car_s) && ((check_car_s-car_s) < 40)) || ((check_car_s<car_s) && ((check_car_s-car_s) > -10)))
                	{
               			lane_change_left = false;
               			
	            	}

```
Next, a check on to see if car ahead in the current lane is getting too close (slowing down or braking) is determined. Based on lane change safety flags, a determination to make a lane change to left or right is made.

If lane change is not safe as determined by the safety flags, speed of the vehilce is reduced to avoid collision. Care is also taken to avoid drastic reduction in speed (Jerk).

If car ahead is far off / no car is ahead, the speed of the vehicle is updated to match highway speed limit (50 mph). Note: **49 mph is used as max velocity** in the program to avoid accidently crossing speed limits momentarily.

### Step 2: Define a trajectory.

To create smooth transition from previous trajectory, waypoints from previous paths are used. If there are no waypoints left in the previous path, reference starting points are created using the car's current reference state. 

```
// Reference states (x,y,yaw)
          double ref_x =car_x;
          double ref_y =car_y;
          double ref_yaw = deg2rad(car_yaw);
```
Next waypoints 30m, 60m , 90m ahead of the starting reference points are added to the waypoint path.

```
// Add waypoints 30m, 60m, 90m ahead of the starting reference points
          vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y); 
```

The waypoint path is shifted and rotated to start at origin and at yaw angle zero (transformation from vehicle's coordinates to global coordinates).
A spline is used to define the future points in this waypoint path. Using a set target distance of say 30 m and using the spline, the y coordinates of the trajectory (for equal incremental intervals of x coordinates (distance travelled in incremental 20ms intervals @reference velocity) are calculated and added to the waypoint path.

The waypoint path is transformed back to vehicle's coordinate system & then added to the previous path (for smooth transition). In this way final trajectory is constructed every 20 ms.

```
// Do transformation back to vehicle's coordinate system
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
```

## Results

Using the path planner the vehcile is found to safely maneuver the enitre highway for 10 miles without any incidents (collision, jerk, max acceleration, outside the lane, crossing speed limits etc.)

[Output Video.mp4](https://youtu.be/F5pEBSL9GWM)

![Image cropped to region of interest](https://github.com/ashsiv/Path-Planning-/blob/master/output%20images/10%20miles.JPG)

As it can be seen, the **vehicle safely travelled 10.03 miles in 13 minutes!** For an **ideal case** of travelling @49mph for 13 minutes straight (without braking), the distance covered will be **10.6 miles**. 

Thus the path planner's **time/fuel efficieny** is also found to be good in addition to safety.

---

## Running the code

### Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./path_planning
