# CarND Path Planner Project



The goal of this self driving car path planner project is to drive collision free in the Unity Udacity simulator, under 50 miles per hour, stay with the traffic, change lanes when the car in front of the car in the right lane is going too slow, for at least 4.32 miles.  This was completed with the challenge of adding waypoints to the file provided.  Collisions were avoided by sensing cars in front in the car’s lane and the speed of the preceding car.  If the vehicle is too close to the preceding car, decelerate at 5 mph each 20 ms.  If not too close and the velocity is less than 50 miles per hour, accelerate at 5 mph each 20 ms.  Below is are details of the algorithm and code for this path planning project.

1.	Read in a data file named highway_map.csv that includes x, y, s dx and dy.  The car position x values are put into a map_waypoints_x vector,  the car position y values into a map_waypoints_y,  vector, the forward distance s values into map_waypoints_s , the change in x, dx, in map_waypoints_dx, and the change in y in map_waypoints_dy.

2.	The length of one time around the simulator track is given as 6945.554 miles

3.	Start in the middle lane, lane = 1 and reference velocity is 0.0 mph to avoid jerking.

4.	Send to the simulator a reference to lane, reference velocity, map_waypoints_x, map_waypoints_y, map_waypoints_dx and map_waypoints_dy variables.

5.	Use micro web socket, uws, to connect to the simulator and read in the car x, y, s, d, yaw, speed, previous_path_x, previous_path_y, and the previous path’s s and d values, end_path_s and end_path_d.  It also reads in the sensor fusion data that is size 12 vector.

6.	Sensor Fusion Data, a list of all other car's attributes on the same side of the road.   A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.  
Look for the cars in the same lane as the car.  If it is the same lane check the speed of the car in the lane and the s distance of the car from the car.  Determine where the car will be in the future, using the in 20 ms time interval during points, the number of previous points size and the speed of the car.
		
7.	If the car is too close to the car in front of it, reduce the velocity by 0.224 every 20 ms.

8.	If the speed is under 49.5 increase the velocity by 0.224 every 20 ms.

9.	Declare ptsx and ptsy vectors for anchor data.

10.	The previous size is assigned to an integer value.  On start up of the simulator the previous size will be zero, less than two.  In this case assign the car x ,y and previous car x, y values to the anchor ptsx[0], ptsy[0], ptsx[1] and ptsy[1].

11.  The next previous size path will be larger than 2 so ptsx[0], ptsy[0], ptsx[1] and ptsy[1] will be defined by previous path points.

12.  The ptsx, ptsy vectors are updated and sent to the simulator as the next x, next y values.  The feedback from the simulator and the code allows the car to driving around the track without collisions, changing lanes, for over 4.32 miles.

In completing this project, I learned that additional waypoints allow the car to not stop at 4.17 miles as it does without extending the waypoints.  Using the ‘cout’ function adds latency and may get collisions.  I verified that the car_s provided by the simulator matches a getFrenet function that takes in the car_x, car_y, theta, (yaw value from simulator), map_waypoints_x and map_waypoints_y.  

