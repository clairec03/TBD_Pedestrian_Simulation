// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) by Christian Gloor

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <stdbool.h>
#include <string>
#include <cstring>

#include <ros/ros.h>

/*
#include <libpedsim_original/ped_agent.h>
#include <libpedsim_original/ped_obstacle.h>
#include <libpedsim_original/ped_waypoint.h>
#include <libpedsim_original/ped_scene.h>
*/

#include "../../3rdparty/libpedsim_original/include/ped_agent.h"
#include "../../3rdparty/libpedsim_original/include/ped_obstacle.h"
#include "../../3rdparty/libpedsim_original/include/ped_waypoint.h"
#include "../../3rdparty/libpedsim_original/include/ped_scene.h"
#include "../../3rdparty/libpedsim_original/include/ped_includes.h"
#include "../../3rdparty/libpedsim_original/include/ped_outputwriter.h"

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
#include <vector>

#include <QApplication>

using namespace std;

string frame = "odom";

std_msgs::Header createMsgHeader() {
  std_msgs::Header msg_header;
  msg_header.stamp = ros::Time::now();
  msg_header.frame_id = frame;
  return msg_header;
}

int main(int argc, char *argv[]) {

    ros::Duration one_sec(1.0);

    printf("Executing line %d\n", __LINE__);
    // create an output writer which will send output to a file 
    Ped::OutputWriter *ow = new Ped::FileOutputWriter();
    ow->setScenarioName("Example 01");

    cout << "PedSim Example using libpedsim version " << Ped::LIBPEDSIM_VERSION << endl;

    printf("Executing line %d\n", __LINE__);
    // Setup
    Ped::Tscene *pedscene = new Ped::Tscene(-200, -200, 400, 400);

    printf("Executing line %d\n", __LINE__);
    // pedscene->setOutputWriter(ow); // SEGFAULTS

    printf("Executing line %d\n", __LINE__);
    Ped::Twaypoint *w1 = new Ped::Twaypoint(-100, 0, 24);
    Ped::Twaypoint *w2 = new Ped::Twaypoint(+100, 0, 12);

    printf("Executing line %d\n", __LINE__);

    // args to Ped::Tobstacle()
    // 1. x coordinate of the first corner of the obstacle.
    // 2. y coordinate of the first corner of the obstacle.
    // 3. x coordinate of the second corner of the obstacle.
    // 4. y coordinate of the second corner of the obstacle.
    Ped::Tobstacle *o1 = new Ped::Tobstacle(0, 0,  -2, -2);
    Ped::Tobstacle *o2 = new Ped::Tobstacle(3, -3,  5, -5);
    pedscene->addObstacle(o1);
    pedscene->addObstacle(o2);

    printf("Executing line %d\n", __LINE__);
    for (int i = 0; i<10; i++) {
	printf("On iteration %d\n", i);
    	printf("Executing line %d\n", __LINE__);
        Ped::Tagent *a = new Ped::Tagent();

    	printf("Executing line %d\n", __LINE__);
	printf("Tagent a is %p\n", (void *) a);
        // a->addWaypoint(w1); // SEGFAULTS
        // a->addWaypoint(w2); // SEGFAULTS

    	printf("Executing line %d\n", __LINE__);
//        a->setPosition(-50 + rand()/(RAND_MAX/80)-40, 0 + rand()/(RAND_MAX/20) -10, 0);
	a->setPosition(i, i, 0);
    	printf("Executing line %d\n", __LINE__);
        pedscene->addAgent(a);

    	printf("Executing line %d\n", __LINE__);

    }

    printf("Executing line %d\n", __LINE__);
    ros::init(argc, argv, "pedsim_simulator");
    ros::NodeHandle nh;

    printf("Executing line %d\n", __LINE__);
    pedsim_msgs::LineObstacles allObs;
    pedsim_msgs::LineObstacle array[2];

    printf("Executing line %d\n", __LINE__);
//    std::vector<Ped::Tobstacle *> obs = pedscene->getAllObstacles();
//    for (std::vector<Ped::Tobstacle *>::iterator it = obs.begin(); it != obs.end(); ++it) {
    for (int i = 0; i < 2; i++) {
	printf("On iteration %d\n", i);
	pedsim_msgs::LineObstacle obs;

    	printf("Executing line %d\n", __LINE__);
    	geometry_msgs::Point start;
	start.x = -i;
	start.y = -i;
	start.z = 0;
	
    	printf("Executing line %d\n", __LINE__);
    	geometry_msgs::Point end;
	end.x = -i+1;
	end.y = -i+1;
	end.z = 0;
	obs.start = start;
	obs.end = end;
	array[i] = obs;

    }


    printf("Executing line %d\n", __LINE__);
    std::vector<pedsim_msgs::LineObstacle> linesAllObs (array, array + sizeof(array) / sizeof(pedsim_msgs::LineObstacle));

    printf("Executing line %d\n", __LINE__);
    for (std::vector<pedsim_msgs::LineObstacle>::iterator it = linesAllObs.begin(); it != linesAllObs.end(); ++it) {
	allObs.obstacles.push_back(*it);
    }
    printf("Executing line %d\n", __LINE__);
    allObs.header = createMsgHeader();

    printf("Executing line %d\n", __LINE__);
    pedsim_msgs::AgentStates allAgents;
    allAgents.header = createMsgHeader();
    pedsim_msgs::AgentState listOfAgents[5];

    printf("Executing line %d\n", __LINE__);
    for (int i = 0; i < 10; i++){
	pedsim_msgs::AgentState currAgent;
	currAgent.header = createMsgHeader();
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
	allAgents.agent_states.push_back(currAgent);
    }
	
    printf("Executing line %d\n", __LINE__);

    pedsim_msgs::AgentGroups allGroups;
    allGroups.header = createMsgHeader();
    pedsim_msgs::AgentGroup listOfGroups[1];

    printf("Executing line %d\n", __LINE__);
    pedsim_msgs::AgentGroup group0;
    group0.header = createMsgHeader();
    group0.group_id = 0;
    group0.age = 0;
    unsigned int peds[5];
   
    printf("Executing line %d\n", __LINE__);
    for (int i = 0; i < 5; i++) {
	pedsim_msgs::AgentState agent = listOfAgents[i];
	peds[i] = agent.id;
    }

    printf("Executing line %d\n", __LINE__);
    geometry_msgs::Pose center;
    geometry_msgs::Point ctrPos;
    geometry_msgs::Quaternion ctrOrientn;

    printf("Executing line %d\n", __LINE__);
    ctrPos.x = 0;
    ctrPos.y = 0;
    ctrPos.z = 0;
    ctrOrientn.x = 0;
    ctrOrientn.y = 0;
    ctrOrientn.z = 0;
    ctrOrientn.w = 0;
    center.position = ctrPos;
    center.orientation = ctrOrientn;

    printf("Executing line %d\n", __LINE__);
    group0.center_of_mass = center;
    listOfGroups[0] = group0;
 //   allGroups.groups[0] = listOfGroups[0];
    allGroups.groups.push_back(group0);

    printf("Executing line %d\n", __LINE__);
    pedsim_msgs::TrackedPersons allPeds;
    allPeds.header = createMsgHeader();
    pedsim_msgs::TrackedPerson allTracks[5];
    std::vector<pedsim_msgs::TrackedPerson> tracks;

    printf("Executing line %d\n", __LINE__);
    // Constant zero covariance matrix
    double covar[36];
    for (int i = 0; i < 36; i++) {
        covar[i] = 0;
    }
	
    printf("Executing line %d\n", __LINE__);
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
        allPeds.tracks.push_back(allTracks[i]);
    } 
    printf("Executing line %d\n", __LINE__);
    // Hard code a robot position
    nav_msgs::Odometry pos;
    geometry_msgs::PoseWithCovariance botPose;
    geometry_msgs::Pose p;
    geometry_msgs::Point botPoint;
    botPoint.x = 0;
    botPoint.y = 0;
    botPoint.z = 0;
    p.position = botPoint;
    geometry_msgs::Quaternion q;
    q.x = 0;
    q.y = 0;
    q.z = 0;
    q.w = 0;
    p.orientation = q;
    botPose.pose = p;
    geometry_msgs::TwistWithCovariance botTwist;
    for (int i = 0; i < 36; i++) {
//    botPose.covariance.push_back(covar[i]);
//    botTwist.covariance.push_back(covar[i]);
	botPose.covariance[i] = covar[i];
	botTwist.covariance[i] = covar[i];
    }
    printf("Executing line %d\n", __LINE__);
    geometry_msgs::Twist simpleTwist;
    geometry_msgs::Vector3 v;
    v.x = 0;
    v.y = 0;
    v.z = 0;
    simpleTwist.linear = v;
    simpleTwist.angular = v;
    botTwist.twist = simpleTwist;

    printf("Executing line %d\n", __LINE__);
   pos.child_frame_id = frame;
   pos.pose = botPose;
   pos.twist = botTwist;

    printf("Executing line %d\n", __LINE__);
   pedsim_msgs::Waypoints allWaypoints;
   allWaypoints.header = createMsgHeader();
   pedsim_msgs::Waypoint pt;

    printf("Executing line %d\n", __LINE__);
   string name = "ALLWAYPOINTS"; 
   pt.name = name;
   pt.behavior = 0;
   pt.position = botPoint; // which is (0, 0, 0)
   pt.radius = 1;
   allWaypoints.waypoints.push_back(pt);

    printf("Executing line %d\n", __LINE__);
    ros::Publisher pub_obstacles_ = 
        nh.advertise<pedsim_msgs::LineObstacles>("/pedsim_simulator/simulated_walls", 1);
    ros::Publisher pub_agent_states_ =
	nh.advertise<pedsim_msgs::AgentStates>("/pedsim_simulator/simulated_agents", 1);
    ros::Publisher pub_agent_groups_ = 
	nh.advertise<pedsim_msgs::AgentGroups>("/pedsim_simulator/simulated_groups", 1);
//   ros::Publisher pub_robot_position_ =
//	nh.advertise<nav_msgs::Odometry>("/pedsim_simulator/robot_position", 1);
    ros::Publisher pub_waypoints_ =
	nh.advertise<pedsim_msgs::Waypoints>("/pedsim_simulator/simulated_waypoints", 1);

    printf("Executing line %d\n", __LINE__);

    // Move all agents for 700 steps (and write their position through the outputwriter)
    for (int i=0; i<700; ++i) {
	printf("On iteration %d\n", i);
	pub_obstacles_.publish(allObs);
        // pedscene->moveAgents(0.3);
	std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    printf("Executing line %d\n", __LINE__);

    while (ros::ok()) {

//	pub_obstacles_.publish(allObs);

	pub_agent_states_.publish(allAgents);
	pub_agent_groups_.publish(allGroups);
//	pub_robot_position_.publish(pos);
	pub_waypoints_.publish(allWaypoints);
	

	// pub_robot_position_.publish(); // No data atm
	// pub_waypoints_.publish(); 
	// No data atm => can try to use pedsim_original to configure waypoints, 
	// such as how they are used at the beginning of this file
	ros::spinOnce();
    }
   
    // Cleanup
    for (Ped::Tagent* agent : pedscene->getAllAgents()) delete agent;
    delete pedscene;
    delete w1;
    delete w2;
    delete o1;
    delete o2;
//    delete ow;


    return EXIT_SUCCESS;
}
