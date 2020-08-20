#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <cmath>

float pi= 3.14159265;

double corrector(double x, double y){
if (x < 1){
	if (y > 0) {
		return y;
		}
	if (y <= 0) {
		return y+360;
		}}
if (x > 1){
	return 180-y;
	}
}

double counter(double angle){
	double interval= 360.0/1440.0;
	double count = angle/interval;
	double intpart, fractpart;
	fractpart = modf(count, &intpart);
	return intpart ;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;


  ros::Publisher robot_wheels_vel =
    node.advertise<geometry_msgs::Vector3>("rotation_left_wheel", 100);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("base_link", "left_wheel",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Quaternion position;
    position.x = transform.getRotation().x();
    position.y = transform.getRotation().y();
    position.z = transform.getRotation().z();
    position.w = transform.getRotation().w();

    //robot_wheels_vel.publish(position);

    geometry_msgs::Vector3 euler_angles;
    


   tf::Quaternion q(position.x, position.y, position.z, position.w);
   tf::Matrix3x3 m(q);
   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);
   euler_angles.x = roll*180/pi;
   euler_angles.y = pitch*180/pi;
   euler_angles.z = yaw*180/pi;
   euler_angles.y = counter(corrector(euler_angles.x,euler_angles.y));
   robot_wheels_vel.publish(euler_angles);
    rate.sleep();
  }
  return 0;
};
