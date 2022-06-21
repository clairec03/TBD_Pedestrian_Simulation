// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) by Christian Gloor

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <stdbool.h>

#include <ros/ros.h>

/*
#include <pedsim/ped_agent.h>
#include <pedsim/ped_obstacle.h>
#include <pedsim/ped_waypoint.h>
#include <pedsim/ped_scene.h> 
*/

//#include "ped_include.h"

#include "ped_agent.h"
#include "ped_obstacle.h"
#include "ped_waypoint.h"
#include "ped_scene.h"

//#include <pedsim_simulator/simulator.h>

#include <pedsim_msgs/AgentForce.h>
#include <pedsim_msgs/AgentGroup.h>
#include <pedsim_msgs/AgentGroups.h>
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/LineObstacle.h>
#include <pedsim_msgs/LineObstacles.h>
#include <pedsim_msgs/Waypoint.h>
#include <pedsim_msgs/Waypoints.h>
#include <pedsim_msgs/TrackedPersons.h>
#include <pedsim_msgs/TrackedPerson.h>
#include <pedsim_msgs/TrackedGroups.h>
#include <pedsim_msgs/TrackedGroup.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>

#include <QApplication>

using namespace std;

int main(int argc, char *argv[]) {

    ros::Duration one_sec(1.0);

    // create an output writer which will send output to a file 
//    Ped::OutputWriter *ow = new Ped::FileOutputWriter();
//    ow->setScenarioName("Example 01");

    //cout << "PedSim Example using libpedsim version " << Ped::LIBPEDSIM_VERSION << endl;

    // Setup
    Ped::Tscene *pedscene = new Ped::Tscene(-200, -200, 400, 400);

//    pedscene->setOutputWriter(ow);

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

    ros::NodeHandle nh;

    pedsim_msgs::LineObstacles allObs;
    //pedsim_msgs::LineObstacle[] array = new pedsim_msgs::LineObstacle[2];
    pedsim_msgs::LineObstacle array[2];

    for (int i = 0; i < 2; i++) {
	pedsim_msgs::LineObstacle obs;
    	geometry_msgs::Point start;
	start.x = 0;
	start.y = 0;
	start.z = 0;
    	geometry_msgs::Point end;
	end.x = i;
	end.y = i;
	end.z = i;
	obs.start = start;
	obs.end = end;
	array[i] = obs;
	allObs.obstacles[i] = array[i];
    }

    //allObs.obstacles = array;

    pedsim_msgs::AgentStates allAgents;
    pedsim_msgs::AgentState listOfAgents[5];

    for (int i = 0; i < 5; i++){
	pedsim_msgs::AgentState currAgent;
	currAgent.id = i;
	unsigned int id = i;
	unsigned int type = 0;
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
	agentPose.orientation = quat;
	
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
	allAgents.agent_states[i] = listOfAgents[i];
    }
	
    //allAgents.agent_states = listOfAgents;

    pedsim_msgs::AgentGroups allGroups;
    pedsim_msgs::AgentGroup listOfGroups[1];

    pedsim_msgs::AgentGroup group0;
    group0.group_id = 0;
    group0.age = 0;
    unsigned int peds[5];
   
    for (int i = 0; i < 5; i++) {
	pedsim_msgs::AgentState agent = listOfAgents[i];
	peds[i] = agent.id;
    }

    geometry_msgs::Pose center;
    geometry_msgs::Point ctrPos;
    geometry_msgs::Quaternion ctrOrientn;

    ctrPos.x = 0;
    ctrPos.y = 0;
    ctrPos.z = 0;
    ctrOrientn.x = 0;
    ctrOrientn.y = 0;
    ctrOrientn.z = 0;
    ctrOrientn.w = 0;
    center.position = ctrPos;
    center.orientation = ctrOrientn;

    group0.center_of_mass = center;
    listOfGroups[0] = group0;
    allGroups.groups[0] = listOfGroups[0];

    pedsim_msgs::TrackedPersons allPeds;
    pedsim_msgs::TrackedPerson allTracks[5];

    // Constant zero covariance matrix
    double covar[36];
    for (int i = 0; i < 36; i++) {
        covar[i] = 0;
    }
	
    for (pedsim_msgs::AgentState agent : listOfAgents) {
	unsigned int i = agent.id;
	pedsim_msgs::TrackedPerson peep;
	peep.track_id = i;
	peep.is_occluded = false;
	peep.is_matched = true;
	peep.detection_id = i;
	peep.age = one_sec;
	geometry_msgs::PoseWithCovariance poseWithCovar;
	geometry_msgs::TwistWithCovariance twistWithCovar;
	poseWithCovar.pose = agent.pose;
	twistWithCovar.twist = agent.twist;
        for (int j = 0; j < 36; j++) {
	    poseWithCovar.covariance[j] = covar[j];
	    twistWithCovar.covariance[j] = covar[j];
        }
	peep.pose = poseWithCovar;
	peep.twist = twistWithCovar;
	allTracks[i] = peep;
        allPeds.tracks[i] = allTracks[i];
    } 

    //allPeds.tracks = allTracks;
    
    pedsim_msgs::TrackedGroups tkdGroups;    
    pedsim_msgs::TrackedGroup listOfTkdGroups[1];

    pedsim_msgs::TrackedGroup grp;
    grp.group_id = group0.group_id; 
    grp.age = one_sec;

    geometry_msgs::PoseWithCovariance groupPoseWithCovar;    
    geometry_msgs::Pose groupPose;
    geometry_msgs::Point groupPt;
    groupPt.x = 2;
    groupPt.y = 2;
    groupPt.z = 2;
    geometry_msgs::Quaternion grpOrientn;
    grpOrientn.x = 0;
    grpOrientn.y = 0;
    grpOrientn.z = 0;
    grpOrientn.w = 0;
    groupPose.position = groupPt;
    groupPose.orientation = grpOrientn; 
    groupPoseWithCovar.pose = groupPose;
    for (int i = 0; i < 36; i++) {
    	groupPoseWithCovar.covariance[i] = covar[i];
    }

    grp.centerOfGravity = groupPoseWithCovar;
    grp.track_ids = group0.members;
	
    listOfTkdGroups[0] = grp;
    tkdGroups.groups[0] = listOfTkdGroups[0];
    

    ros::Publisher pub_obstacles_ = 
        nh.advertise<pedsim_msgs::LineObstacles>("simulated_walls", 1);
    ros::Publisher pub_agent_states_ =
	nh.advertise<pedsim_msgs::AgentStates>("simulated_agents", 1);
    ros::Publisher pub_agent_groups_ = 
	nh.advertise<pedsim_msgs::AgentGroups>("simulated_groups", 1);
    ros::Publisher pub_robot_position_ =
	nh.advertise<nav_msgs::Odometry>("robot_position", 1);
    ros::Publisher pub_waypoints_ =
	nh.advertise<pedsim_msgs::Waypoints>("simulated_waypoints", 1);

    while (ros::ok()){
    	pub_obstacles_.publish(allObs);
	pub_agent_states_.publish(allAgents);
//	pub_robot_position_.publish();
//	pub_waypoints_.publish();

	// Not needed: 
	// ros::spinOnce();
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
//    delete ow;


    return EXIT_SUCCESS;
}
