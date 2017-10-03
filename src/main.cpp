#define _USE_MATH_DEFINES  // to use M_PI constant
#include <fstream>
#include <cmath>
#include <uWS\uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"
#include "spline.h"
// comment is not using on windows use_ipv4
#define  use_ipv4

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{
	// Returns the closest waypoint relative to
	// the car position
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	// get the closest waypoint to x, y and define it closestWaypoint
	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);
	// use the index of the closestWaypoint to define the closest
	// map_x and closest map_y values.
	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];
	// the angle between (map_y - y) and (map_x - x)
	// is defined as heading
	double heading = atan2((map_y - y), (map_x - x));
	// the new angle is the absolute difference between 
	// theta and the angle between (map_y - y) and
	// (map_x - x) and is defined as angle
	// 
	double angle = abs(theta - heading);
	// of the angle is less than 45 degrees
	// increment the closestWaypoint
	if (angle > pi() / 4)
	{
		closestWaypoint++;
	}
	// return the next Waypoint
	return closestWaypoint;

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;
	
	while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s*cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s*sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return { x,y };

}

int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "data/highway_map.csv";//TODO
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		double s;
		double d_x;
		double  d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}
	// start in lane 1
	int lane = 1;
	// reference velocity to target
	double ref_vel = 2.0;

	h.onMessage([&lane, &ref_vel, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
		uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
					int prev_size = previous_path_x.size();
					// 
					if (prev_size > 0)
					{
						car_s = end_path_s;
					}
					else
					{
						cout << " prev_size is less than zero " << endl;
					}

					//if (car_s == end_path_s)
					//{
					//	car_s = 0.0;
					//}
					bool too_close = false;
					// find ref_v to use
					// Sensor fusion size is 12
					for (int i = 0; i < sensor_fusion.size(); i++)
					{
						// car is in lane
						double d = sensor_fusion[i][6];
						if (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2))
						{// d value tells if the car is in our lane if is
						 // check speed
							double vx = sensor_fusion[i][3];
							double vy = sensor_fusion[i][4];
							double check_speed = sqrt(vx*vx + vy*vy);
							// check car's s value
							double check_car_s = sensor_fusion[i][5];
							// where the car will be s in the future
							check_car_s += ((double)prev_size*0.02*check_speed);
							// are we close to car in front of us,
							// check_car_s less than 30m
							if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
							{
								too_close = true;
								if (lane > 0)
									lane = 0; // get in the left lane
							}

						}
					}
					//
					if (too_close)
					{
						ref_vel -= .224;
						//ref_vel -= .336;
					}
					else if (ref_vel < 49.5)
					{
						ref_vel += .224;
					}

					// ptsx, ptsy will be anchor points
					vector<double> ptsx;
					vector<double> ptsy;
					//read in the car's global x and y position
					// and assign it to ref x and ref y
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);

					if (prev_size < 2)
					{
						// this happens once at the beginning
		
						double prev_car_x = car_x - cos(car_yaw);
						double prev_car_y = car_y - sin(car_yaw);
						ptsx.push_back(prev_car_x);
						ptsx.push_back(car_x);
						ptsy.push_back(prev_car_y);
						ptsy.push_back(car_y);
						
						cout << " At initial start of car simulation " << endl;
						cout << " ptsx[1]  is " << car_x << " , " << car_y << endl;
						cout << " ptsx[0] is  " << prev_car_x << " , " << prev_car_y << endl;
						cout << "car_s is " << car_s << endl;
					}
					else
					{
						ref_x = previous_path_x[prev_size - 1];
						ref_y = previous_path_y[prev_size - 1];

						double ref_x_prev = previous_path_x[prev_size - 2];
						double ref_y_prev = previous_path_y[prev_size - 2];
						ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
						


						ptsx.push_back(ref_x_prev);
						ptsx.push_back(ref_x);
						ptsy.push_back(ref_y_prev);
						ptsy.push_back(ref_y);
						
					}
					// in frenet add evenly 30m spaced points ahead of the start
					// In freenet add evenly the spaced points
					
					vector<double> next_wp0 = getXY(car_s + 30, (2.0 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp1 = getXY(car_s + 60, (2.0 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp2 = getXY(car_s + 90, (2.0 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
 
					ptsx.push_back(next_wp0[0]);
					ptsx.push_back(next_wp1[0]);
					ptsx.push_back(next_wp2[0]);

					ptsy.push_back(next_wp0[1]);
					ptsy.push_back(next_wp1[1]);
					ptsy.push_back(next_wp2[1]);

					
					// now we have 5 points on ptsy ptsx
					// 2 previous points and 30, 60, 90
					for (size_t i = 0; i < ptsx.size(); i++)
					{
						// shift car reference angle to 0 degrees 
						// at car coordinates 0,0
						double shift_x = ptsx[i] - ref_x;
						double shift_y = ptsy[i] - ref_y;
						// shift to car coordinate, 0,0 0 degrees
						ptsx[i] = (shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
						ptsy[i] = (shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
					}
					if (car_s < 100)
					{
						int i = 0;
						for (auto xvec : ptsx)
						{
							cout << " car_s is  " << car_s << endl;
							cout << "  ptsx[" << i << "]  " << xvec;
							i++;
						}
						cout << endl;
						int j = 0;
						for (auto yvec : ptsy)
						{
							cout << "  ptsy[" << j << "]  " << yvec;
							j++;
						}
						cout << endl;
					}
					if (car_s > 6814)
					{
						int i = 0;
						for (auto xvec : ptsx)
						{
							cout << " car_s is  " << car_s << endl;
							cout << "  ptsx[" <<i<<"]  " << xvec;
							i++;
						}
						cout << endl;
						int j = 0;
						for (auto yvec : ptsy)
						{
							cout << "  ptsy[" << j << "]  " << yvec;
							j++;
						}
						cout << endl;
					}
					
					tk::spline s;

					// set x y points to the spline

					s.set_points(ptsx, ptsy);

					vector<double> next_x_vals;
					vector<double> next_y_vals;
					
					// Start with all of the previous points
					for (size_t i = 0; i < previous_path_x.size(); i++)
					{
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}
					// Calculate how to break up splines
					// pts so we travel at desired velocity
					double target_x = 30.0;
					double target_y = s(target_x);
					double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

					double x_add_on = 0;  // start at orgin

										  // Fill up the rest of the path planner
										  // after filling it with previous points
										  // what is previous_path
					for (int i = 1; i <= 50 - previous_path_x.size(); i++)
					{
						double N = (target_dist / (0.02*ref_vel / 2.24)); //2.24 to convert to m/s
						double x_point = x_add_on + (target_x) / N;
						// s provides the y value to x
						double y_point = s(x_point);

						x_add_on = x_point;

						double x_ref = x_point;
						double y_ref = y_point;

						// rotate back to normal after rotating
						x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
						y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

						x_point += ref_x;
						y_point += ref_y;

						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);
					}

					json msgJson;
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);

				}
			}
			else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
		char *message, size_t length) {
		ws->close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
#ifdef use_ipv4
	if (h.listen("0.0.0.0", port)) {
#else
	if (h.listen(port)) {
#endif
		std::cout << "Listening to port " << port << std::endl;
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
	}