// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) by Christian Gloor

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <stdbool.h>

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

    pedsim_msgs::AgentGroups allGroups;
    pedsim_msgs::AgentGroup[] listOfGroups = new pedsim_msgs::AgentGroup[1];

    pedsim_msgs::AgentGroup group0;
    group0.group_id = 0;
    group0.age = 0;
    uint64[] peds = new uint64[5];
   
    for (int i = 0; i < 5; i++) {
	pedsim_msgs::AgentState agent = listOfAgent[i];
	peds[i] = agent.id;
    }

    geometry_msgs::Pose center;
    geometry_msgs::Point ctrPos;
    geometry_msgs::Quaternion ctrOrientn;

    center.x = 0;
    center.y = 0;
    center.z = 0;
    ctrOrientn.x = 0;
    ctrOrientn.y = 0;
    ctrOrientn.z = 0;
    ctrOrientn.w = 0;
    center.position = ctrPos;
    center.orientation = ctrOrientn;

    group0.center_of_mass = center;
    listOfGroups[0] = group0;
    allGroups.groups = listOfGroups;

    pedsim_msgs::TrackedPersons allPeds;
    pedsim_msgs::TrackedPerson[] allTracks = new pedsim_msgs::TrackedPerson[5];

    // Constant zero covariance matrix
    float64[] covar = new float64[36];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
	    covar[i][j] = 0;
	}
    }
	
    for (pedsim_msgs::AgentState agent : listOfAgents) {
	uint64 i = agent.id;
	pedsim_msgs::TrackedPerson peep;
	peep.tracked_id = ped;
	peep.is_occluded = false;
	peep.is_matched = true;
	peep.detection_id = ped;
	peep.age = 0;
	geometry_msgs::PoseWithCovariance poseWithCovar;
	geometry_msgs::TwistWithCovariance twistWithCovar;
	poseWithCovar.pose = agent.pose;
	twistWithCovar.twist = agent.twist;
	poseWithCovar.covariance = covar;
	twistWithCovar.covariance = covar;
	peep.pose = poseWithCovar;
	peep.twist = twistWithCovar;
	allTracks[ped] = peep;
    } 

    allPeds.tracks = allTracks;
    
    pedsim_msgs::TrackedGroups tkdGroups;    
    pedsim_msgs::TrackedGroup[] listOfTkdGroups = new pedsim_msgs::TrackedGroup[1];

    pedsim_msgs::TrackedGroup grp;
    grp.group_id = group0.group_id; 
    grp.age = group0.age;

    geometry_msgs::PoseWithCovariance groupPoseWithCovar;    
    geometry_msgs::Pose groupPose;
    geometry_msgs::Point groupPt;
    groupPose.x = 2;
    groupPose.y = 2;
    groupPose.z = 2;
    geometry_msgs::Quaternion grpOrientn;
    grpOrientn.x = 0;
    grpOrientn.y = 0;
    grpOrientn.z = 0;
    grpOrientn.w = 0;
    groupPose.position = groupPt;
    groupPose.orientation = grpOrientn; 
    groupPoseWithCovar.pose = groupPose;
    groupPoseWithCovar.covariance = covar;

    grp.centerOfGravity = groupPoseWithCovar;
    grp.track_ids = group0.members;
	
    listOfTkdGroups[0] = grp;
    tkdGroups.groups = listOfTkdGroups;
    

    ros::nodeHandle nh;

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
    delete ow;


    return EXIT_SUCCESS;
}
