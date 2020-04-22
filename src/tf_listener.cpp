#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transform_point(const tf::TransformListener& listener)
{

  geometry_msgs::PointStamped point;
  point.header.frame_id = "base_point";

  point.header.stamp = ros::Time();

  point.point.x = 0.0;
  point.point.y = 1.0;
  point.point.z = 2.0;

  geometry_msgs::PointStamped base_point;
  listener.transformPoint("base_link", point, base_point);

  ROS_INFO("point: (%.2f, %.2f, %.2f) -> base_link: (%.2f, %.2f, %.2f) at time %.2f",
    point.point.x, point.point.y, point.point.z,
    base_point.point.x, base_point.point.y, base_point.point.z,
    base_point.header.stamp.toSec()
  );

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rvr_tf_listener");

  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transform_point, boost::ref(listener)));

  ros::spin();
}
