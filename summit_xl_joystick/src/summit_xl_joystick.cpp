/*
 * summit_xl_joystick
 * Copyright (c) 2011, Robotnik Automation, SLL
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
 * \brief Allows to use a joystick with the summit_controller, sending the messages received through the joystick input, correctly adapted, to the "summit_controller/command" topic.
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <robotnik_msgs/set_mode.h>
#include <robotnik_msgs/ptz.h>
#include <summit_xl_joystick/enable_disable.h>
//#include <enable_disable.h>

#define DEFAULT_NUM_OF_BUTTONS		16
#define DEFAULT_AXIS_LINEAR_X		1
#define DEFAULT_AXIS_LINEAR_Y		0
#define DEFAULT_AXIS_LINEAR_Z		3
#define DEFAULT_AXIS_ANGULAR		2	
#define DEFAULT_SCALE_LINEAR_X		1.0
#define DEFAULT_SCALE_LINEAR_Y		1.0
#define DEFAULT_SCALE_LINEAR_Z		1.0
#define DEFAULT_SCALE_ANGULAR		2.0
#define NUM_BUTTONS                 20

class SummitJoy
{
public:
  SummitJoy();
  bool EnableDisable(summit_xl_joystick::enable_disable::Request &req, summit_xl_joystick::enable_disable::Response &res );

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_x_, linear_y_, linear_z_, angular_;
  double l_scale_x_, l_scale_y_, l_scale_z_, a_scale_;
  //! It will publish into command velocity (for the robot) and the ptz_state (for the pantilt)
  ros::Publisher vel_pub_;
  ros::Publisher ptz_pub_;
  //! It will be suscribed to the joystick
  ros::Subscriber joy_sub_;
  //! Name of the topic where it will be publishing the velocity
  std::string cmd_topic_vel_;
  //! Name of the service where it will be modifying the digital outputs
  std::string cmd_service_io_;
  //! Name of the topic where it will be publishing the pant-tilt values	
  std::string cmd_topic_ptz_;
  double current_vel;
  //! Number of the DEADMAN button
  int dead_man_button_;
  //! Number of the button for increase or decrease the speed max of the joystick	
  int speed_up_button_, speed_down_button_;
  int button_output_1_, button_output_2_;
  int output_1_, output_2_;
  bool bOutput1, bOutput2;  
  //! Enables/disables the pad
  ros::ServiceServer enable_disable_srv_;
  //! button to change kinematic mode
  int button_kinematic_mode_;
  //! kinematic mode
  int kinematic_mode_;
  //! Service to modify the kinematic mode
  ros::ServiceClient setKinematicMode;  
  //! Name of the service to change the mode
  std::string cmd_set_mode_;
  //! buttons to the pan-tilt camera
  int ptz_tilt_up_, ptz_tilt_down_, ptz_pan_right_, ptz_pan_left_;
  //! Pan & tilt increment (degrees)
  int pan_increment_, tilt_increment_;
  //! Service to modify the digital outputs
  ros::ServiceClient modbus_write_do_client;  
  //! Number of buttons of the joystick
  int num_of_buttons_;
  //! Pointer to a vector for controlling the event when pushing the buttons
  bool bRegisteredButtonEvent[NUM_BUTTONS];
  //! Flag to enable/disable the communication with the publishers topics
  bool bEnable;

};

SummitJoy::SummitJoy():
  linear_x_(1),
  linear_y_(0),
  angular_(2),
  linear_z_(3)
{

	current_vel = 0.1;
	// 
	nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
	// MOTION CONF
	nh_.param("axis_linear_x", linear_x_, DEFAULT_AXIS_LINEAR_X);
    nh_.param("axis_linear_y", linear_y_, DEFAULT_AXIS_LINEAR_Y);
    nh_.param("axis_linear_z", linear_z_, DEFAULT_AXIS_LINEAR_Z);
	nh_.param("axis_angular", angular_, DEFAULT_AXIS_ANGULAR);
	nh_.param("scale_angular", a_scale_, DEFAULT_SCALE_ANGULAR);
	nh_.param("scale_linear_x", l_scale_x_, DEFAULT_SCALE_LINEAR_X);
	nh_.param("scale_linear_y", l_scale_y_, DEFAULT_SCALE_LINEAR_Y);
	nh_.param("scale_linear_z", l_scale_z_, DEFAULT_SCALE_LINEAR_Z);
	nh_.param("cmd_topic_vel", cmd_topic_vel_, cmd_topic_vel_);
	nh_.param("button_dead_man", dead_man_button_, dead_man_button_);
	nh_.param("button_speed_up", speed_up_button_, speed_up_button_);  //4 Thrustmaster
	nh_.param("button_speed_down", speed_down_button_, speed_down_button_); //5 Thrustmaster
		
	// DIGITAL OUTPUTS CONF
	nh_.param("cmd_service_io", cmd_service_io_, cmd_service_io_);
	nh_.param("button_output_1", button_output_1_, button_output_1_);
	nh_.param("button_output_2", button_output_2_, button_output_2_);
	nh_.param("output_1", output_1_, output_1_);
	nh_.param("output_2", output_2_, output_2_);
	// PANTILT CONF
	nh_.param("cmd_topic_ptz", cmd_topic_ptz_, cmd_topic_ptz_);
	nh_.param("button_ptz_tilt_up", ptz_tilt_up_, ptz_tilt_up_);
	nh_.param("button_ptz_tilt_down", ptz_tilt_down_, ptz_tilt_down_);
	nh_.param("button_ptz_pan_right", ptz_pan_right_, ptz_pan_right_);
	nh_.param("button_ptz_pan_left", ptz_pan_left_, ptz_pan_left_);
	nh_.param("pan_increment", pan_increment_, 1);
	nh_.param("tilt_increment",tilt_increment_, 1);

	// KINEMATIC MODE 
	nh_.param("button_kinematic_mode", button_kinematic_mode_, button_kinematic_mode_);
	nh_.param("cmd_service_set_mode", cmd_set_mode_, cmd_set_mode_);
	kinematic_mode_ = 1;



	//bRegisteredButtonEvent = new bool(num_of_buttons_);	
	for(int i = 0; i < NUM_BUTTONS; i++){
		bRegisteredButtonEvent[i] = false;
	}

	ROS_INFO("Topic PTZ = [%s]", cmd_topic_ptz_.c_str());
	ROS_INFO("Service set_mode = [%s]", cmd_set_mode_.c_str());
	ROS_INFO("Service I/O = [%s]", cmd_topic_vel_.c_str());
	ROS_INFO("Axis linear X = %d", linear_x_);
	ROS_INFO("Axis linear Y = %d", linear_y_);
	ROS_INFO("Axis linear Z = %d", linear_z_);
	ROS_INFO("Axis angular = %d", angular_);
	ROS_INFO("Scale angular = %f", a_scale_);
	ROS_INFO("Deadman button = %d", dead_man_button_);
	ROS_INFO("OUTPUT1 button %d", button_output_1_);
	ROS_INFO("OUTPUT2 button %d", button_output_2_);
	ROS_INFO("OUTPUT1 button %d", button_output_1_);
	ROS_INFO("OUTPUT2 button %d", button_output_2_);
	ROS_INFO("Kinematic mode button %d", button_kinematic_mode_);


  	// Publish through the node handle Twist type messages to the summit_xl_ctrl/command topic
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);

	//  Publishes msgs for the pant-tilt cam
	ptz_pub_ = nh_.advertise<robotnik_msgs::ptz>(cmd_topic_ptz_, 1);

 	// Listen through the node handle sensor_msgs::Joy messages from joystick 
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &SummitJoy::joyCallback, this);
	
  	// Request service to activate / deactivate digital I/O
  	// modbus_write_do_client = nh_.serviceClient<modbus_io::write_digital_output>(cmd_service_io_);
	bOutput1 = bOutput2 = false;

	// Request service to set kinematic mode 
	setKinematicMode = nh_.serviceClient<robotnik_msgs::set_mode>(cmd_set_mode_);

	// Advertises new service to enable/disable the pad
	enable_disable_srv_ = nh_.advertiseService("/summit_xl_joystick/enable_disable",  &SummitJoy::EnableDisable, this);

	bEnable = true;	// Communication flag enabled by default

}

/*
 *	\brief Enables/Disables the pad
 *
 */
bool SummitJoy::EnableDisable(summit_xl_joystick::enable_disable::Request &req, summit_xl_joystick::enable_disable::Response &res )
{
	bEnable = req.value;

	ROS_INFO("SummitXLJoystick::EnablaDisable: Setting to %d", req.value);
	res.ret = true;
	return true;
}

void SummitJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist vel;
	robotnik_msgs::ptz ptz;
    bool ptzEvent = false;
	
  	// Actions dependant on dead-man button
 	if (joy->buttons[dead_man_button_] == 1) {
		//ROS_ERROR("SummitJoy::padCallback: DEADMAN button %d", dead_man_button_);
		// Set the current velocity level
		if ( joy->buttons[speed_down_button_] == 1 ){

			if(!bRegisteredButtonEvent[speed_down_button_]) 
				if(current_vel > 0.1){
		  			current_vel = current_vel - 0.1;
					bRegisteredButtonEvent[speed_down_button_] = true;
					 ROS_INFO("Velocity: %f%%", current_vel*100.0);	
				}	 	
		}else{
			bRegisteredButtonEvent[speed_down_button_] = false;
		}
		 
		if (joy->buttons[speed_up_button_] == 1){
			if(!bRegisteredButtonEvent[speed_up_button_])
				if(current_vel < 0.9){
					current_vel = current_vel + 0.1;
					bRegisteredButtonEvent[speed_up_button_] = true;
			 	 	ROS_INFO("Velocity: %f%%", current_vel*100.0);
				}
		  
		}else{
			bRegisteredButtonEvent[speed_up_button_] = false;
		}
		 
		vel.angular.x = current_vel*(a_scale_*joy->axes[angular_]);
		vel.angular.y = current_vel*(a_scale_*joy->axes[angular_]);
		vel.angular.z = current_vel*(a_scale_*joy->axes[angular_]);
		vel.linear.x = current_vel*l_scale_x_*joy->axes[linear_x_];
		vel.linear.y = current_vel*l_scale_y_*joy->axes[linear_y_];
		vel.linear.z = current_vel*l_scale_z_*joy->axes[linear_z_];

		// LIGHTS
		if (joy->buttons[button_output_1_] == 1) {

			if(!bRegisteredButtonEvent[button_output_1_]){
				//ROS_INFO("SummitJoy::padCallback: OUTPUT1 button %d", button_output_1_);
				//modbus_io::write_digital_output modbus_wdo_srv;
				//modbus_wdo_srv.request.output = output_1_;
				bOutput1=!bOutput1;
				//modbus_wdo_srv.request.value = bOutput1;
				//modbus_write_do_client.call( modbus_wdo_srv );
				bRegisteredButtonEvent[button_output_1_] = true;
			}
		}else{
			bRegisteredButtonEvent[button_output_1_] = false;
		}

		if (joy->buttons[button_output_2_] == 1) {

			if(!bRegisteredButtonEvent[button_output_2_]){
				//ROS_INFO("SummitJoy::padCallback: OUTPUT2 button %d", button_output_2_);
				//modbus_io::write_digital_output modbus_wdo_srv;
				//modbus_wdo_srv.request.output = output_2_;
				bOutput2=!bOutput2;
				//modbus_wdo_srv.request.value = bOutput2;
				//modbus_write_do_client.call( modbus_wdo_srv );
				bRegisteredButtonEvent[button_output_2_] = true;
			}		  	
		}else{
			bRegisteredButtonEvent[button_output_2_] = false;
		}
		 
		// PTZ CAMERA
		// TILT-MOVEMENTS (RELATIVE POS)
		// TILT-MOVEMENTS (RELATIVE POS)
		ptz.pan = ptz.tilt = ptz.zoom = 0.0;
		ptz.relative = true;
		if (joy->buttons[ptz_tilt_up_] == 1) {		
			if(!bRegisteredButtonEvent[ptz_tilt_up_]){
				ptz.tilt = tilt_increment_;
				//ROS_INFO("SummitXLPad::padCallback: TILT UP");
				bRegisteredButtonEvent[ptz_tilt_up_] = true;
				ptzEvent = true;
			}
		}else {
			bRegisteredButtonEvent[ptz_tilt_up_] = false;
		}

		if (joy->buttons[ptz_tilt_down_] == 1) {
			if(!bRegisteredButtonEvent[ptz_tilt_down_]){
			  	ptz.tilt = -tilt_increment_;
				//ROS_INFO("SummitXLPad::padCallback: TILT DOWN");
				bRegisteredButtonEvent[ptz_tilt_down_] = true;
				ptzEvent = true;
			}
		}else{
			bRegisteredButtonEvent[ptz_tilt_down_] = false;
		}
		 
		// PAN-MOVEMENTS (RELATIVE POS)
		if (joy->buttons[ptz_pan_left_] == 1) {			
			if(!bRegisteredButtonEvent[ptz_pan_left_]){
				ptz.pan = -pan_increment_;
				//ROS_INFO("SummitXLPad::padCallback: PAN LEFT");
				bRegisteredButtonEvent[ptz_pan_left_] = true;
				ptzEvent = true;
			}
		}else{
			bRegisteredButtonEvent[ptz_pan_left_] = false;
		}

		if (joy->buttons[ptz_pan_right_] == 1) {
			if(!bRegisteredButtonEvent[ptz_pan_right_]){
			  	ptz.pan = pan_increment_;
				//ROS_INFO("SummitXLPad::padCallback: PAN RIGHT");
				bRegisteredButtonEvent[ptz_pan_right_] = true;
				ptzEvent = true;
			}
		}else{
			bRegisteredButtonEvent[ptz_pan_right_] = false;
		}
		
		  	
		if (joy->buttons[button_kinematic_mode_] == 1) {

			if(!bRegisteredButtonEvent[button_kinematic_mode_]){
				// Define mode (inc) - still coupled
				kinematic_mode_ += 1;
				if (kinematic_mode_ > 2) kinematic_mode_ = 1;
 				ROS_INFO("SummitXLJoy::joyCallback: Kinematic Mode %d ", kinematic_mode_);
				// Call service 
				robotnik_msgs::set_mode set_mode_srv;
				set_mode_srv.request.mode = kinematic_mode_;
				setKinematicMode.call( set_mode_srv );
				bRegisteredButtonEvent[button_kinematic_mode_] = true;
			}
		}else{
			bRegisteredButtonEvent[button_kinematic_mode_] = false;
		}

	}
   	else {
		vel.angular.x = 0.0;	vel.angular.y = 0.0; vel.angular.z = 0.0;
		vel.linear.x = 0.0; vel.linear.y = 0.0; vel.linear.z = 0.0;
	}

    if (ptzEvent) ptz_pub_.publish(ptz);

	vel_pub_.publish(vel);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "summit_xl_joystick");
	SummitJoy summit_joy;
	ros::spin();
}

