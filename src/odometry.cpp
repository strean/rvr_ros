

int main(int argc, char** argv)
{






  while ( ros::ok() )
  {

    time = ros::Time::now();



    double dt = ( time - time_last ).toSec();
    double delta_x = ( vx * cos(th) - vy * sin(th) ) * dt;
    double delta_y = ( vx * sin(th) + vy * cos(th) ) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw( 0, 0, th );

    odom_trans.header.stamp = time;
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw( th );

    nav_msgs::Odometry odom;
    odom.header.stamp = time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = vth;

    time_last = time;

    broadcaster.sendTransform( odom_trans );
    publisher.publish( odom );

    loop_rate.sleep();
  }

  return 0;
}
