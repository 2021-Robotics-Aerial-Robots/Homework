#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// Callback of timer
// Broadcaster transform from A to B and B to C
void broadcastTF(const ros::TimerEvent& timer_event)
{
  static tf::TransformBroadcaster br;
  tf::Transform tf_A_B, tf_B_C, tf_C_D, tf_D_A1;
  tf::Quaternion q_A_B,  q_B_C, q_C_D, q_D_A1;
  tf::Vector3     v_A_B, v_B_C, v_C_D, v_D_A1;

  v_A_B.setValue(1.0, 0.0, 0.0);
  //use RPY set value of quaternion
  //then use quaternion to rotate
  q_A_B.setRPY(0, 0, M_PI/2);

  v_B_C.setValue(1, 0, 0);
  q_B_C.setRPY(0, 0, M_PI);

  v_C_D.setValue(0, -1, 0);
  q_C_D.setRPY(0, 0, M_PI);

  v_D_A1.setValue(-1, 0 , 0.5);
  q_D_A1.setRPY(0, 0, M_PI);

  tf_A_B.setOrigin(v_A_B);
  tf_A_B.setRotation(q_A_B);

  tf_B_C.setOrigin(v_B_C);
  tf_B_C.setRotation(q_B_C);

  tf_C_D.setOrigin(v_C_D);
  tf_C_D.setRotation(q_C_D);

  tf_D_A1.setOrigin(v_D_A1);
  tf_D_A1.setRotation(q_D_A1);


  br.sendTransform(tf::StampedTransform(tf_A_B, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "A", // paranet frame ID
                                        "B")); // child frame ID

  br.sendTransform(tf::StampedTransform(tf_B_C,
                                        ros::Time::now(),
                                        "B",
                                        "C"));

  br.sendTransform(tf::StampedTransform(tf_C_D,
                                        ros::Time::now(),
                                        "C",
                                        "D"));

  br.sendTransform(tf::StampedTransform(tf_D_A1,
                                        ros::Time::now(),
                                        "D",
                                        "A1"));

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ncrl_tf_learning");
  ros::NodeHandle nh;
  // Create timer with 2.0 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), broadcastTF);
  while (ros::ok()){ros::spinOnce();}
  return 0;
}
