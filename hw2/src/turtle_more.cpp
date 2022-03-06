#include "ros/ros.h"
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <string>
#include <iostream>

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "spawnturtles");
    ros::NodeHandle n;

    // Check if the service is on
    ros::service::waitForService("/turtlesim/spawn");

    // remove the original turtle
    ros::ServiceClient client_kill=n.serviceClient<turtlesim::Kill>("/turtlesim/kill");
    turtlesim::Kill kill_name;
    kill_name.request.name = "turtle1";
    client_kill.call(kill_name); 

    // spawn turtles
    ros::ServiceClient client_spawn = n.serviceClient<turtlesim::Spawn>("/turtlesim/spawn");
    turtlesim::Spawn turtle;

    // define your names of turtles
    string names[] = {"follower1","follower2","leader"};
    size_t length = sizeof(names) / sizeof(names[0]);

    // define spawn position
    for(int i = 0; i< length ;i++){
        turtle.request.name = names[i];
        turtle.request.x = 2*(i+1);
        turtle.request.y = 2*(i+1);
        turtle.request.theta = 0;
        client_spawn.call(turtle);
    }

}