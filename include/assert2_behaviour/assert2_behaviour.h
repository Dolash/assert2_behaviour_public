#ifndef ASSERT2_BEHAVIOUR_H
#define ASSERT2_BEHAVIOUR_H

#include <ros/ros.h>
#include <cmath>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <p2os_msgs/GripperState.h>
#include <std_msgs/Float32.h>
#include <p2os_msgs/SonarArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/ColorRGBA.h>
#include <autonomy_leds_msgs/Keyframe.h>
#include <vector>
#include <unistd.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/MapMetaData.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <rosgraph_msgs/Clock.h>

#define PI 3.14159
#define TWO_PI 6.283185*/

class Assert2Behaviour {
private:

	ros::Time timer;
	rosgraph_msgs::Clock clockTime;

	/*Controls the loop rate*/
 	double loopHz;
	bool stageMode;
	
	bool poseReceived;
	bool subjectPoseReceived;
	
	bool returnTrip;
	bool firstGoal;
	bool doorReached;
	
	bool subjectDetected;
	
	bool fighting;
    bool navigating;
	
	std::string poseTopic;
	std::string subjectPoseTopic;
	std::string cmdVelTopic;
	std::string scanTopic;
	std::string multiplierTopic;
	
	float startX;
    float startY;
	
    float goalX;
    float goalY;

	float startYaw;
	float goalYaw;

    float startDoorX;
    float startDoorY;
	
    float goalDoorX;
    float goalDoorY;

	float startDoorYaw;
	float goalDoorYaw;

    float doorX;
    float doorY;

	float currentGoalX;
    	float currentGoalY;
	
		bool clockReceived;
		
			std_msgs::Bool emergencyPause;
			std_msgs::Float32 multiplier;

geometry_msgs::PoseStamped goal_cmd;

    bool laserReceived;
	bool firstTime;

	std::vector<float> latestLaserScan;
	
	void stagePoseCallback(const nav_msgs::Odometry::ConstPtr& pose);
  	void laserCallback(const sensor_msgs::LaserScan scanData);
	void stageSubjectPoseCallback(const nav_msgs::Odometry::ConstPtr& pose);
	void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& pose);
	void viconSubjectCallback(const geometry_msgs::TransformStamped::ConstPtr& pose);
	void clockCallback(const rosgraph_msgs::Clock clockData);
		void emergencyStopCallback(const std_msgs::Bool emergencyPauseData);
	
	/*for vicon*/
	geometry_msgs::TransformStamped latestPoses;
    geometry_msgs::TransformStamped latestSubjectPoses;

	/*for stage*/
	nav_msgs::Odometry stagePose;
	nav_msgs::Odometry stageSubjectPose;
	
	void waypointing();
	float getDesiredAngle(float targetX, float targetY, float currentXCoordinateIn, float currentYCoordinateIn);
	void subjectAhead();
	void navigatingBehaviour();
	void fightingBehaviour();
	
protected:
  ros::NodeHandle nh;
  ros::NodeHandle privNh;

   	/*Subscriber for pose*/
 	ros::Subscriber poseSub;
 	/*Subscriber for subject's pose*/
 	ros::Subscriber subjectPoseSub;
	
	 ros::Subscriber laserSub;
	
	ros::Publisher cmd_vel_pub;
	ros::Publisher multiplier_pub;
	
	ros::Subscriber clockSub;
	
ros::Publisher goal_pub;

	/*Movement orders for Pioneer*/
	geometry_msgs::Twist move_cmd;

	
		ros::Subscriber emergencyStopSub;

 	
public:
  Assert2Behaviour(ros::NodeHandle& nh);
  ~Assert2Behaviour();

  virtual void spin();
  virtual void spinOnce();

}; // class Assert2Behaviour

#endif // ASSERT2_BEHAVIOUR_H
