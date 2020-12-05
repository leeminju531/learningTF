// how to create coordinate frame
// we wanna broadcast the changing coordinate frames of the turtle.

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr &msg){
	// help make the task of publishing transforms easier
	static tf2_ros::TransformBroadcaster br;

	// this message is mostly used by the tf package.
	geometry_msgs::TransformStamped transformStamped; 

	transformStamped.header.stamp = ros::Time::now();//we need to give the transform being published a timestamp, in this case current time
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = turtle_name;
	transformStamped.transform.translation.x = msg->x;
	transformStamped.transform.translation.y = msg->y;
	transformStamped.transform.translation.z = 0.0;
	tf2::Quaternion q;
	q.setRPY(0,0,msg->theta);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped); // this is where real work is done
}

int main(int argc, char** argv){
	ros::init(argc, argv, "my_tf2_broadcaster");

	//NodeHandle ->starting and shutting down a node as simple

	ros::NodeHandle private_node("~");
	if(!private_node.hasParam("turtle")){
		if(argc != 2) {ROS_ERROR("need turtle name as argument"); return -1;}
		turtle_name = argv[1];
	}
	else{
		private_node.getParam("turtle",turtle_name);
	}

	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe(turtle_name+"/pose",10,&poseCallback);

	ros::spin();
	return 0;

}