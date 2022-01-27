#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// Callback of timer
// Broadcaster transform from A to B and B to C
void broadcastTF(const ros::TimerEvent& timer_event)
{
  static tf::TransformBroadcaster br;
  tf::Transform tf_A_B, tf_B_C;
  tf::Quaternion q_A_B,  q_B_C;
  tf::Vector3     v_A_B, v_B_C;
  v_A_B.setValue(1.0, 0.0, 0.0);
  q_A_B.setRPY(M_PI/2, M_PI/2, M_PI/4);

  v_B_C.setValue(0.0, 0.5, 0.5);
  q_B_C.setRPY(0, 0, M_PI);

  tf_A_B.setOrigin(v_A_B);
  tf_A_B.setRotation(q_A_B);
  tf_B_C.setOrigin(v_B_C);
  tf_B_C.setRotation(q_B_C);

  br.sendTransform(tf::StampedTransform(tf_A_B, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "A", // paranet frame ID
                                        "B")); // child frame ID

  br.sendTransform(tf::StampedTransform(tf_B_C,
                                        ros::Time::now(),
                                        "B",
                                        "C"));

  tf::Quaternion q;
  tf::Transform tf_A_A1, tf_A1_A2, tf_A1_A3, tf_A1_A4;
  tf_A_A1.setOrigin(tf::Vector3(0.0, 1.0, 0.0));
  q.setRPY(0, 0, 0);
  tf_A_A1.setRotation(q);

  tf_A1_A2.setOrigin(tf::Vector3(0.3, 0.0, 0.0));
  q.setRPY(0, 0, M_PI/4);
  tf_A1_A2.setRotation(q);

  tf_A1_A3.setOrigin(tf::Vector3(0.6, 0.0, 0.0));
  q.setRPY(0, M_PI/2, M_PI/4);
  tf_A1_A3.setRotation(q);

  tf_A1_A4.setOrigin(tf::Vector3(1.0, 0.0, 0.0));
  q.setRPY(M_PI/2, M_PI/2, M_PI/4);
  tf_A1_A4.setRotation(q);

  br.sendTransform(tf::StampedTransform(tf_A_A1,
                                        ros::Time::now(),
                                        "A",
                                        "A1"));
  br.sendTransform(tf::StampedTransform(tf_A1_A2,
                                        ros::Time::now(),
                                        "A1",
                                        "A2"));
  br.sendTransform(tf::StampedTransform(tf_A1_A3,
                                        ros::Time::now(),
                                        "A1",
                                        "A3"));
  br.sendTransform(tf::StampedTransform(tf_A1_A4,
                                        ros::Time::now(),
                                        "A1",
                                        "A4"));

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
