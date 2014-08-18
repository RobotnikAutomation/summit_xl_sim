/*
 * summit_xl_waypoints
 * Copyright (c) 2013, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik Automation, SLL
 * \brief Allows to send a set of waypoints to the move_base controller 
 */

#define DEFAULT_SPEED             0.5

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>

#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"


using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef struct {
   std::string name;
   std::string frame_id;
   geometry_msgs::Pose2D position2d;
   bool sent;
   bool reached;
} waypoint_t;

class SummitWaypoints
{

private:
  
  //! Diagnostics
  self_test::TestRunner self_test_;
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  double desired_freq_;
  diagnostic_updater::Updater diagnostic_;			// General status diagnostic updater
  diagnostic_updater::FrequencyStatus freq_diag_;		         // Component frequency diagnostics
  diagnostic_updater::HeaderlessTopicDiagnostic *subs_command_freq; // Topic reception frequency diagnostics
  ros::Time last_command_time;					// Last moment when the component received a command
  // diagnostic_updater::FunctionDiagnosticTask command_freq_;

  //! Node running
  bool running;

public:

  //! Error counters and flags 
  int error_count_;
  std::string was_slow_;
  std::string error_status_;

  //! Reference link to send the goals
  std::string reference_link_;

  //! Loop waypoints 
  bool loop_;

  //! Service add waypoint
  ros::ServiceServer srv_AddWaypoint_;

  //! Service to enable / disable Joystick
  ros::ServiceClient enable_disable_joystick;  

  //! Name of the service enable / disable Joystick
  std::string cmd_enable_disable_;

  //actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;  
  MoveBaseClient ac_; 
  
  // Waypoint array (not limited)
  std::vector<waypoint_t> wp_array_;

/*!	\fn SummitWaypoints()
 * 	\brief Public constructor
*/
SummitWaypoints(ros::NodeHandle h) : self_test_(), diagnostic_(),
  node_handle_(h), private_node_handle_("~"), 
  error_count_(0),
  desired_freq_(10),
  freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05)   ),
  ac_("move_base", true)   //tell the action client that we want to spin a thread by default
  {
    running = false;
    
    ros::NodeHandle summit_xl_wp_node_handle(node_handle_, "summit_xl_waypoints");
    // Parameters 
    // Reference link frame 
    private_node_handle_.param<std::string>("reference_link", reference_link_, "base_link");

    // Publishing
    // status_pub_ = summit_xl_wp_node_handle.advertise<xxx_msgs::XXX>("/status", 50);

    // Advertise services
    // srv_AddWaypoints_ = summit_xl_wp_node_handle.advertiseService("add_waypoint", &SummitWaypoints::srvCallback_AddWaypoint, this);

    // Component frequency diagnostics
    diagnostic_.setHardwareID("summit_xl_waypoints");
    diagnostic_.add("Waypoints diagnostic", this, &SummitWaypoints::wp_diagnostic);
    diagnostic_.add( freq_diag_ );    
  }

/*!	\fn bool ConnectToActionServer()
 * 	\brief 
*/
bool ConnectToActionServer()
{
  //wait for the action server to come up
  while(!ac_.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }
  return true;
}

bool SendGoal( int iWaypoint )
{
  // Check if no more waypoints 
  if (iWaypoint > wp_array_.size() ) {
	ROS_INFO("summit_xl_waypoints::SendGoal - no more WPs i=%d", iWaypoint);
	return false;
	}

  // Only send waypoints once
  if (wp_array_[ iWaypoint ].sent ) return false;

  // Prepare message
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = wp_array_[ iWaypoint ].frame_id;
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = wp_array_[ iWaypoint ].position2d.x;
  goal.target_pose.pose.position.y = wp_array_[ iWaypoint ].position2d.y;
  // wp_array_[ iWaypoint ].position2d.theta;  // check orientation ?
  // TODO convert theta to quaternion.
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("SummitWaypoints::SendGoal - Sending goal");
  ac_.sendGoal(goal);

  wp_array_[ iWaypoint ].sent = true;  

  return true;
}

bool Control( int iWaypoint )
{
  // Check if no more waypoints 
  if (iWaypoint > wp_array_.size() ) {
	ROS_INFO("summit_xl_waypoints::Control - no more WPs i=%d", iWaypoint);
	return false;
	}

  ac_.waitForResult(ros::Duration(1.0) );

  if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Reached waypoint %d", iWaypoint);
    wp_array_[iWaypoint].reached = true;
    return true;
    }
  else {
    // ROS_INFO("The base failed to move forward 1 meter for some reason");
    return false;
    }
}


/*
 *\brief Checks the status of the component. Diagnostics
 *
 */
void wp_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	/*
	stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Controller ready");
	stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Controller initialing");
	stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Controller on Failure or unknown state");
	// add and addf are used to append key-value pairs.
	stat.add("State FLW", driver->GetMotorFlwStateString()); // Internal controller state
        */ 
}

/*
 *	\brief Reads yaml file with the defined waypoints
 *
 */
bool parse_waypoint_yaml_file()
{
  waypoint_t wp;

  // Read waypoints list from yaml file
  XmlRpc::XmlRpcValue nodeList;

  // Parse yaml file 
  bool loop;
  if (node_handle_.getParam("loop", loop)) 
	ROS_INFO("Loop = %d", (int) loop);

  XmlRpc::XmlRpcValue waypoint_array;
  if(node_handle_.getParam("waypoints", waypoint_array)){
      if(waypoint_array.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < waypoint_array.size(); ++i){
	     ROS_INFO("waypoint_array[i].getType = %d", waypoint_array[i].getType() );         
	     if(waypoint_array[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            	if(waypoint_array[i].hasMember("frame_id") && waypoint_array[i].hasMember("position")){	
                  //read positions 
                  std::string name = waypoint_array[i]["name"];		 
		     wp.name = name;
                  std::string frame_id = waypoint_array[i]["frame_id"];		 
		     wp.frame_id = frame_id;
		  double x = waypoint_array[i]["position"][0];		  
		  double y = waypoint_array[i]["position"][1];		
		  double theta = waypoint_array[i]["position"][2];
		  wp.position2d.x = x;
		  wp.position2d.y = y;
		  wp.position2d.theta = theta;
		  wp.reached = false;
 		  wp.sent = false;
		  wp_array_.push_back( wp );		 
		  ROS_INFO("wp[%d] name=%s frame_id=%s [x=%5.2f, y=%5.2f, theta=%5.2f]", 
				i, name.c_str(), frame_id.c_str(), x, y, theta); 		
	          }
                } 
             }
         }
      }

}

}; // class SummitWaypoints


int main(int argc, char** argv){

  ros::init(argc, argv, "summit_xl_waypoints");
 
  ros::NodeHandle n;		
  
  // Create object
  SummitWaypoints summit_waypoints(n);

  // Read waypoints from file
  summit_waypoints.parse_waypoint_yaml_file();

  ROS_INFO("Connecting to action server");

  // Connect to action server
  summit_waypoints.ConnectToActionServer();

  ros::Rate r(10.0);

  // Current goal
  int iCurrentWaypoint = 0;

  ROS_INFO("Entering main loop");

  // Main Loop
  while( ros::ok() ){

        // Send new wp
	summit_waypoints.SendGoal( iCurrentWaypoint );
  
	// Check if waypoint has been reached 
	//if (summit_waypoints.Control( iCurrentWaypoint ) )
	//	iCurrentWaypoint++;

	ros::spinOnce();
	r.sleep();
	}

  return 0;
}

