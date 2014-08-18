#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {

ros::init(argc, argv, "summit_xl_joint_state");
ros::NodeHandle n;
ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
tf::TransformBroadcaster broadcaster;
ros::Rate loop_rate(30);

const double degree = M_PI/180;
double rot4 = 90;

geometry_msgs::TransformStamped odom_trans;
sensor_msgs::JointState joint_state;
odom_trans.header.frame_id = "odom";
odom_trans.child_frame_id = "base_link";

joint_state.name.resize(11);
joint_state.position.resize(11);
joint_state.name[0] ="joint_front_right_steer";
joint_state.name[1] ="joint_front_right_wheel";
joint_state.name[2] ="joint_front_left_steer";
joint_state.name[3] ="joint_front_left_wheel";
joint_state.name[4] ="joint_back_left_steer";
joint_state.name[5] ="joint_back_left_wheel";
joint_state.name[6] ="joint_back_right_steer";
joint_state.name[7] ="joint_back_right_wheel";
joint_state.name[8] ="scissor_bars1_joint";
joint_state.name[9] ="scissor_bars2_joint";
joint_state.name[10] ="scissor_prismatic_joint";

while (ros::ok()) {
    //update joint_state
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = 0;
    joint_state.position[1] = 0;
    joint_state.position[2] = 0;
    joint_state.position[3] = 0;
    joint_state.position[4] = 0;
    joint_state.position[5] = 0;
    joint_state.position[6] = 0;
    joint_state.position[7] = 0;
    joint_state.position[8] = rot4*degree;
	joint_state.position[9] = 0;
	joint_state.position[10] = rot4 / 180;

    // update transform
    // (moving in a circle with radius=2)
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

    //send the joint state and transform
    joint_pub.publish(joint_state);
    broadcaster.sendTransform(odom_trans);

    rot4 += 1;
    if (rot4 > 90) rot4 = 0;

    loop_rate.sleep();
}
return 0;
}
