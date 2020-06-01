#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "math.h"

using namespace std;
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// Define Start Lane and Initial Reference Velocity
int lane = 1; // Middle lane
double ref_vel = 0.0; // Set start reference velocity to zero to avoid max jerk during time step 0.
// Variable to check time interval between two lane changes.
int elapsed=0;
int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
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

  

  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          // Previous path waypoints
          int prev_size = previous_path_x.size();
          
          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visitcd
           *   sequentially every .02 seconds
           */
          // time counter - used to check interval between lane changes
          
          // STEP 1:  Determine ACTION to take based on Sensor Fusion data
          elapsed=elapsed+1;
            if(prev_size > 0)
          {
            car_s = end_path_s;
          }
          // Define action state varibles
          bool too_close = false;
          bool lane_change_left =true;
          bool lane_change_right=true;
          bool changed = false;
          
          // Decide action for every 'i'th car data from Sensor Fusion
          for(int i=0; i <sensor_fusion.size();i++)
          {
            float d = sensor_fusion[i][6];
            
            //If car is in right most lane avoid lane shift to right
            if(lane==2)
            {
              lane_change_right=false;
            }
             //If car is in leftmost lane avoid lane shift to left
            if(lane==0)
            {
              lane_change_left=false;
            }
            
            if(lane!=0)
            {
            //check if neighbouring car is in left lane (within 6 meters)
            if(d<car_d && car_d-d <6)
            	{   
                    
              		double vx = sensor_fusion[i][3];
             		double vy = sensor_fusion[i][4];
            		double check_speed = sqrt(vx*vx + vy*vy);
            	    double check_car_s = sensor_fusion[i][5];
                    check_car_s += ((double)prev_size*0.02*check_speed);
            	
             // check if the neighbouring car (ahead or behind) is in proximity. (40m safe gap is chosen for front car and 10m safe gap is chosen                for car behind to do safe lane change)
             if(((check_car_s>car_s) && ((check_car_s-car_s) < 40)) || ((check_car_s<car_s) && ((check_car_s-car_s) > -10)))
                	{
               			lane_change_left = false;
               			
	            	}
              
            	}
            }
            
            //check if neighbouring car in right lane (within 6 meters)
            if(lane!=2)
            {
             if(d>car_d && d-car_d < 6)
            	{
              		double vx = sensor_fusion[i][3];
             		double vy = sensor_fusion[i][4];
            		double check_speed = sqrt(vx*vx + vy*vy);
            	    double check_car_s = sensor_fusion[i][5];
                    check_car_s += ((double)prev_size*0.02*check_speed);
            	
             // check if the neighbouring car (ahead or behind) is in proximity. (40m safe gap is chosen for front car and 10m safe gap is chosen                for car behind to do safe lane change)
             if(((check_car_s>car_s) && ((check_car_s-car_s) <40)) || ((check_car_s<car_s) && ((check_car_s-car_s) > -10)))
                	{
               			lane_change_right = false;
               			
	            	}
           
            	}
            }
            
            // Check if car ahead in the current lane is too close
            if(d<(2+4*lane+2)&& d>(2+4*lane-2))
            {
              
             double vx = sensor_fusion[i][3];
             double vy = sensor_fusion[i][4];
             double check_speed = sqrt(vx*vx + vy*vy);
             double check_car_s = sensor_fusion[i][5];
              
             check_car_s += ((double)prev_size*0.02*check_speed);
             
             if((check_car_s>car_s) && ((check_car_s-car_s) < 30))
             {
               
              too_close = true;
              cout<<"Too close"<<endl;

             }
            }
          }
          
          // If car ahead in the current lane is slowing down and it is safe to change lane to left and time has elapsed from last lane change      		  (100*20ms =2s), change lane to left.
          if(too_close && lane_change_left && elapsed >100)
          {
            elapsed=0;
             cout<<"Too close and shift left"<<endl;
            if(lane!=0)
            {
            lane=lane-1;
            }
          }
          // If car ahead in the current lane is slowing down and it is safe to change lane to right and time has elapsed from last lane change      		  (100*20ms =2s), change lane to right.
          else if(too_close && lane_change_right && elapsed >100)
          {
            elapsed=0;
            cout<<"Too close and shift right"<<endl;
            if(lane !=2)
            {
            lane=lane+1;
            }
          }
          // If not safe to make lane change, slow down
          else if(too_close)
          {
            ref_vel -=0.5;
            cout<<"Too close reduce speed"<<endl;
          } 
          // If no car is ahead, increase speed to match highway speed limit
          else if(ref_vel <49.0)
          {
            ref_vel +=0.5;
            cout<<"Increase speed"<<endl;
          }
      
          // STEP 2:  Construct the Waypoint path.
          // Create Waypoints
          vector<double> ptsx;
          vector<double> ptsy;
          
          // Reference states (x,y,yaw)
          double ref_x =car_x;
          double ref_y =car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // If previous path is close to empty use car as starting reference
          if(prev_size < 2)
          {
            // Create additional point back in time (1-step before) based on the current yaw angle of the car
            double prev_car_x =car_x -cos(car_yaw);
            double prev_car_y =car_y -sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          
          }
          
          // If points in previous path is available use last and second last points in previous path as reference
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev); // Calculate angle between the two previous points
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          
          }
          
          // Add waypoints 30m, 60m, 90m ahead of the starting reference points
          vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y); 
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // Transforming the waypoint path to origin and set starting yaw angle to zero (shift and rotation from vehicle's cordinates to global coordinates)
          for(int i=0;i<ptsx.size();i++)
          {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }
          // Define the spline
          tk::spline s;
          // Add point to spline
          s.set_points(ptsx,ptsy);
          
          //Future path
          vector<double> next_x_vals;
          vector<double> next_y_vals;
		 
          // For smooth transition add previous path points (if any) while building the future path
          for(int i=0; i <previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // For a given target 'x' distance calculate the target 'y' point using spline
          double target_x =30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
          double x_add_on =0;
          
	 // 'y' coordinates for equal incremental intervals of x coordinates (distance travelled in incremental 20ms intervals @reference velocity) are calculated and added to the waypoint path
          for(int i=1; i<=50-previous_path_x.size(); i++)
          {
            
            double N = (target_dist/(0.02*ref_vel/2.24)); // 0.02 --> Car will visit every waypoint in 20 ms @ reference velocity. Factor 2.24 is               for m/s to mph conversion. 
            double x_point = x_add_on+(target_x)/N;
            //Calculate y point using the spline
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Do transformation back to vehicle's coordinate system
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
            
            x_point +=ref_x;
            y_point +=ref_y;
           
            // Push the transformed points to the previous path (This is the final trajectory)
           next_x_vals.push_back(x_point);
           next_y_vals.push_back(y_point);     
          }


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
