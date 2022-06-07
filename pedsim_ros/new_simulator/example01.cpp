// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) by Christian Gloor

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>

// Beginning of publisher header files
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
// End of publisher header files

#include "ped_includes.h"
#include "ped_outputwriter.h"

using namespace std;

int main(int argc, char *argv[]) {

    // create an output writer which will send output to a file 
    Ped::OutputWriter *ow = new Ped::FileOutputWriter();
    ow->setScenarioName("Example 01");

    cout << "PedSim Example using libpedsim version " << Ped::LIBPEDSIM_VERSION << endl;

    // Setup
    Ped::Tscene *pedscene = new Ped::Tscene(-200, -200, 400, 400);

    pedscene->setOutputWriter(ow);

    Ped::Twaypoint *w1 = new Ped::Twaypoint(-100, 0, 24);
    Ped::Twaypoint *w2 = new Ped::Twaypoint(+100, 0, 12);

    Ped::Tobstacle *o = new Ped::Tobstacle(0, -50,  0, +50);
    pedscene->addObstacle(o);

    for (int i = 0; i<10; i++) {
        Ped::Tagent *a = new Ped::Tagent();

        a->addWaypoint(w1);
        a->addWaypoint(w2);

        a->setPosition(-50 + rand()/(RAND_MAX/80)-40, 0 + rand()/(RAND_MAX/20) -10, 0);

        pedscene->addAgent(a);
    }

    // Move all agents for 700 steps (and write their position through the outputwriter)
    for (int i=0; i<700; ++i) {
        pedscene->moveAgents(0.3);
	std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    // Cleanup
    for (Ped::Tagent* agent : pedscene->getAllAgents()) delete agent;
    delete pedscene;
    delete w1;
    delete w2;
    delete o;
    delete ow;

    ros::NodeHandle nh_;

 
    // Original publishers 
    pub_obstacles_ = 
        nh_.advertise<pedsim_msgs::LineObstacles>("simulated_walls", 1);
    pub_agent_states_ =
	nh_.advertise<pedsim_msgs::AgentStates>("simulated_agents", 1);
    pub_agent_groups_ = 
	nh_.advertise<pedsim_msgs::AgentGroups>("simulated_groups", 1);
    pub_robot_position_ =
	nh_.advertise<nav_msgs::Odometry>("robot_position", 1);
    pub_waypoints_ =
	nh_.advertise<pedsim_msgs::Waypoints>("simulated_waypoints", 1);
   


    pedsim_msgs::LineObstacles list;

    pedsim_msgs::LineObstacle[] array = new pedsim_msgs::LineObstacle[2];

    for (int i = 0; i < 2; i++) {
	pedsim_msgs::LineObstacle obs;
	geometry_msgs::Point start;
	start.x = (float64)-i;
	start.y = (float64)-i;
	start.z = (float64)0;
	geometry_msgs::Point end;
	end.x = (float64)i;
	end.y = (float64)i;
	end.z = (float64)0;
	obs.start = start;
	obs.end = end;
	array[i] = obs;
    }
    list.obstacles = array;


    // Publisher written according to ROS tutorial
    // ...
   // AgentStates

    // AgentGroups


    // Odometry
    
    // Waypoints
    
	
    while (ros::ok()){
	// publish msg
	// ROS_INFO_STREAM("forces:\n" <<)
    	pub_obstacles_.publish(list);
    }


    /*
       // setup ros publishers
  pub_obstacles_ =
      nh_.advertise<pedsim_msgs::LineObstacles>("simulated_walls", queue_size);
  pub_agent_states_ =
      nh_.advertise<pedsim_msgs::AgentStates>("simulated_agents", queue_size);
  pub_agent_groups_ =
      nh_.advertise<pedsim_msgs::AgentGroups>("simulated_groups", queue_size);
  pub_robot_position_ =
      nh_.advertise<nav_msgs::Odometry>("robot_position", queue_size);
  pub_waypoints_ =
    nh_.advertise<pedsim_msgs::Waypoints>("simulated_waypoints", queue_size);
    */

    // example01.cpp has a different simulator system => diff ROS publishers need to be set up

    

    return EXIT_SUCCESS;
}
