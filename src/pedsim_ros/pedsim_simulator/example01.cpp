// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) by Christian Gloor

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>

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

    ros::NodeHandle nh;

    pedsim_msgs::LineObstacles allObs;
    pedsim_msgs::LineObstacle[] array = new pedsim_msgs::LineObstacle[2];

    for (int i = 0; i < 2; i++) {
	pedsim_msgs::LineObstacle obs;
    	geometry_msgs::Point start;
	start.x = (float64)0;
	start.y = (float64)0;
	start.z = (float64)0;
    	geometry_msgs::Point end;
	end.x = (float64)i;
	end.y = (float64)i;
	end.z = (float64)i;
	obs.start = start;
	obs.end = end;
	array[i] = obs;
    }

    allObs.obstacles = array;

    pedsim_msgs::AgentStates allAgents;
    pedsim_msgs::AgentState[] listOfAgents = new pedsim_msgs::AgentState[5];

    for (int i = 0; i < 5; i++){
	pedsim_msgs::AgentState currAgent;
	currAgent.id = (uint64) i;
	uint64 id = i;
	uint16 type = 0;
	currAgent.type = type;
	string state = "individual_moving";
	currAgent.social_state = state;
	geometry_msgs::Pose agentPose;
	geometry_msgs::Point posingPt;
	posingPt.x = i;
	posingPt.y = i;
	posingPt.z = i;
	geometry_msgs::Quaternion quat;
	quat.x = 0;
	quat.y = 0;
	quat.z = 0;
	quat.w = 0;
	agentPose.position = posingPt;
	agentPose.orientation quat;
	
	geometry_msgs::Twist twist;
	geometry_msgs::Vector3 linear;
	geometry_msgs::Vector3 angular;
	linear.x = 0.5;
	linear.y = 0.5;
	linear.z = 0.5;
	angular.x = 0.0;
	angular.y = 0.0;
	angular.z = 0.0;
	twist.linear = linear;
	twist.angular = angular;

	pedsim_msgs::AgentForce force;
	geometry_msgs::Vector3 desired;
	desired.x = 0.5;
	desired.y = 0.5;
	desired.z = 0.5;
	geometry_msgs::Vector3 obstacle;
	obstacle.x = 0.1;
	obstacle.y = 0.1;
	obstacle.z = 0.1;
	geometry_msgs::Vector3 social;
	social.x = 0;
	social.y = 0;
	social.z = 0;
	geometry_msgs::Vector3 coherence;
	coherence.x = 0;
	coherence.y = 0;
	coherence.z = 0;
	geometry_msgs::Vector3 gaze;
	gaze.x = 0;
	gaze.y = 0;
	gaze.z = 0;
	geometry_msgs::Vector3 repulsion;
	repulsion.x = 0;
	repulsion.y = 0;
	repulsion.z = 0;
	geometry_msgs::Vector3 random;
	random.x = 0;
	random.y = 0;
	random.z = 0;
	force.desired_force = desired;
	force.obstacle_force = obstacle;
	force.social_force = social;
	force.group_coherence_force = coherence;
	force.group_gaze_force = gaze;
	force.group_repulsion_force = repulsion;
	force.random_force = random;
	// Setting all fields of currAgent equal to the above parameters
	currAgent.pose = agentPose;
	currAgent.twist = twist;
	currAgent.forces = force; 
	listOfAgents[i] = currAgent;
    }
	
    allAgents.agent_states = listOfAgents;
    

    pub_obstacles_ = 
        nh.advertise<pedsim_msgs::LineObstacles>("simulated_walls", 1);
    pub_agent_states_ =
	nh.advertise<pedsim_msgs::AgentStates>("simulated_agents", 1);
    pub_agent_groups_ = 
	nh.advertise<pedsim_msgs::AgentGroups>("simulated_groups", 1);
    pub_robot_position_ =
	nh.advertise<nav_msgs::Odometry>("robot_position", 1);
    pub_waypoints_ =
	nh.advertise<pedsim_msgs::Waypoints>("simulated_waypoints", 1);
   

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
