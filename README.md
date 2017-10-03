CarND Path Planner Project

1.	 Read in a data file named highway_map.csv that includes x, y, s dx and dy.  The car position x values are put into a map_waypoints_x vector,  the car position y values into a map_waypoints_y,  vector, the forward distance s values into map_waypoints_s , the change in x, dx, in map_waypoints_dx, change in y in map_waypoints_dy.
2.	The length of one time around the simulator track is given as 6945.554 miles
3.	Start in the middle lane, lane = 1 and reference velocity is 0.0 mph
4.	Send to the simulator a reference to lane, reference velocity, map_waypoints_x, map_waypoints_y, map_waypoints_dx and map_waypoints_dy
5.	Use micro web socket, uws, to connect to the simulator and read in the car x, y, s, d, yaw, speed, previous_path_x, previous_path_y, and the previous path’s s and d values, end_path_s and end_path_d.  It also reads in the sensor fusion data that is size 12 vector.
6.	Sensor Fusion Data, a list of all other car's attributes on the same side of the road.   A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.  
Look for the cars in the same lane as the car.  If it is the same lane check the speed of the car in the lane and the s distance of the car from the car.  Determine where the car will be in the future, using the in 20 ms time interval during points, the number of previous points size and the speed of the car.
for (int i = 0; i < sensor_fusion.size(); i++)
{
// car is in lane
d = sensor_fusion[i][6];
if (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2))
{// d value tells if the car is in our lane check speed
double vx = sensor_fusion[i][3];
double vy = sensor_fusion[i][4];
double check_speed = sqrt(vx*vx + vy*vy);
// check car's s value
double check_car_s = sensor_fusion[i][5];
// where the car will be s in the future
check_car_s += ((double)prev_size*0.02*check_speed);
are we close to car in front of us,
// check_car_s less than 30m
if ((check_car_s > car_s) && ((check_car_s - car_s) 
< 30))
{
too_close = true;
if (lane > 0)
				lane = 0; // get in the left lane
		}
	}
}

7.	If the car is too close to the car in front of it, reduce the velocity by 0.224 every 20 ms.
8.	If the speed is under 49.5 increase the velocity by 0.224 every 20 ms.
9.	Declare ptsx and ptsy vectors.
10.	The previous size is assigned to an integer value.  On start up of the simulator the previous size will be zero, less than two.  In this case assign 
