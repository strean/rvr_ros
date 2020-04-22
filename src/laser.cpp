#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<sensor_msgs::LaserScan>("scan", 50);

  unsigned int points = 100;
  double laser_freq = 40;
  double ranges[points];
  double intensities[points];

  int count = 0;
  ros::Rate r(1.0);

  while ( n.ok() )
  {

    for (unsigned int i = 0; i < points; ++i) {
      ranges[i] = count;
      intensities[i] = 100 + count;
    }

    ros::Time scan_time = ros::Time::now();

    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "base_link";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 3.14 / points;
    scan.time_increment = (1 / laser_freq) / points;
    scan.range_min = 0.0;
    scan.range_max = 100.0;

    scan.ranges.resize(points);
    scan.intensities.resize(points);

    for (unsigned int i = 0; i < points; ++i) {
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }

    publisher.publish(scan);
    ++count;
    r.sleep();
  }

}
