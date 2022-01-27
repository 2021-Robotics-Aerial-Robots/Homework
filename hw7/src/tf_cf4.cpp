#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ncrl_tf_listener");
  ros::NodeHandle nh;

  tf::TransformListener listener1;
  tf::TransformListener listener2;
  tf::TransformListener listener3;

  tf::StampedTransform trans_A_B;
  tf::StampedTransform trans_A_C;
  tf::StampedTransform trans_A_D;

  try{
    listener1.waitForTransform("A", "B", ros::Time(0), ros::Duration(3.0));
    listener1.lookupTransform("A", // source_frame
                             "B", // target_frame
                             ros::Time(0),
                             trans_A_B);
    listener2.waitForTransform("A", "C", ros::Time(0), ros::Duration(3.0));
    listener2.lookupTransform("A", // source_frame
                             "C", // target_frame
                             ros::Time(0),
                             trans_A_C);
    listener3.waitForTransform("A", "D", ros::Time(0), ros::Duration(3.0));
    listener3.lookupTransform("A", // source_frame
                             "D", // target_frame
                             ros::Time(0),
                             trans_A_D);
    }
  catch (tf::TransformException ex){
    //ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  // Convert quaternion to RPY
  tf::Quaternion q1(trans_A_B.getRotation().x(),
                   trans_A_B.getRotation().y(),
                   trans_A_B.getRotation().z(),
                   trans_A_B.getRotation().w());
  tf::Matrix3x3 mat1(q1);
  double roll, pitch, yaw;
  mat1.getRPY(roll, pitch, yaw);
  printf("A -> B rotation : %f \t %f \t %f \n", roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);

  tf::Transform tf_B_C = trans_A_B.inverse() * trans_A_C;
  double x, y, z;
  x = tf_B_C.getOrigin().getX();
  y = tf_B_C.getOrigin().getY();
  z = tf_B_C.getOrigin().getZ();
  printf("B -> C translation : %f \t %f \t %f \n",x, y, z);

  tf::Quaternion q2(tf_B_C.getRotation().x(),
                   tf_B_C.getRotation().y(),
                   tf_B_C.getRotation().z(),
                   tf_B_C.getRotation().w());
  tf::Matrix3x3 mat2(q2);
  mat2.getRPY(roll, pitch, yaw);
  printf("B -> C rotation : %f \t %f \t %f \n", roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
  return 0;
}
