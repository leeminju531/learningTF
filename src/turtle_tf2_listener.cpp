#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc,char** argv){
	ros::init(argc,argv,"my_tf2_listener");

	ros::NodeHandle node;

	ros::service::waitForService("spawn");
	ros::ServiceClient spawner = 
		node.serviceClient<turtlesim::Spawn>("spawn");
	turtlesim::Spawn turtle;
	turtle.request.x = 4;
	turtle.request.y = 2;
	turtle.request.theta = 0;
	turtle.request.name = "turtle2";
	spawner.call(turtle);

	ros::Publisher turtle_vel = 
		node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",10);

	tf2_ros::Buffer tfBuffer;
	//like TransformBroadcaster,
	//help make the task of receiving transforms easier
	// Once the listener is created, 
	//it starts receiving tf2 transformations over the wire, and buffers them for up to 10 seconds
	tf2_ros::TransformListener tfListener(tfBuffer);



	ros::Rate rate(10.0);
	while(node.ok()){
		geometry_msgs::TransformStamped transformStamped;
		try{													//turtle2(target Frame) <- turtle1 (from this) :: we can see turtle1 frame at turtle2 frame  
			//transformStamped = tfBuffer.lookupTransform("turtle2","turtle1",ros::Time(0)); // the real work is done
																						//ros::Time(0) will just get us the latest available transform in the buffer

			//transformStamped = tfBuffer.lookupTransform("turtle2","turtle1",ros::Time::now());
			// -> error!! why?? 
			//each listener has a buffer where it stores all the coordinate transforms
			// coming from the different tf2 broadcasters.
			// When a broadcaster sends out a transform, 
			// it takes some time before that transform gets into the buffer
			//you should wait a few milliseconds for that information to arrive in buffer !!


			//The lookupTransform() can take four arguments.
 			transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", 
 													ros::Time::now(),ros::Duration(3.0));
			//The 4th is an optional timeout. It will block for up to that duration waiting for it to timeout.



			//tiem Traveler
			
		}																				
		catch(tf2::TransformException &ex){
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();  // mean sec
			continue; // -> ★★★★★★ in case by doing 'continue', process restart line 36 (because of 'while')
		}

		geometry_msgs::Twist vel_msg;

		vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
											transformStamped.transform.translation.x);
		vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x,2)+
										  pow(transformStamped.transform.translation.y,2));

		turtle_vel.publish(vel_msg);

		rate.sleep();
	}
	return 0;
}