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
#include <autonomy_leds_msgs/Keyframe.h>

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
	bool clearedA;
	bool clearedB;
	bool fightStart;


	float parkX;
	float parkY;
	float parkDiffX;
	float parkDiffY;
	
	bool fighting;
    bool navigating;
	
	bool unpaused;
	bool listeningForUnpause;
	bool clearLights;

	std::string poseTopic;
	std::string subjectPoseTopic;
	std::string cmdVelTopic;
	std::string scanTopic;
	std::string multiplierTopic;
	std::string joyTopic;

	std::string mapFrame;
	
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
	geometry_msgs::Quaternion currentGoalOrientation;

	geometry_msgs::PoseStamped robotPose;
	
	float doorEngageDistance;
	float subjectDoorEngageDistance;
	float winnerMultiple;

	float desiredYaw;

		bool clockReceived;
		bool oneRun;

	float currentVelocity;
	float currentSubjectVelocity;
		
			std_msgs::Bool emergencyPause;
			std_msgs::Float32 multiplier;

geometry_msgs::PoseStamped goal_cmd;

	autonomy_leds_msgs::Keyframe lights;
	std::vector<std_msgs::ColorRGBA> lightColours;
	std_msgs::ColorRGBA tempColor;

    bool laserReceived;
	bool firstTime;

	std::vector<float> latestLaserScan;

	

	/*Set these to whichever sound_player values for different sounds you want and they play at the appropriate times*/
    int startupSound;
    int fightStartSound;
    int loseFightSound;
    int winFightSound;
    int backToNormalSound;
    /*Set these to whichever setLights values for different light arrangements you want and they play at the appropriate times*/
    int startupLights;
    int fightStartLights;
    int loseFightLights;
    int winFightLights;
    int backToNormalLights;
	
	void stagePoseCallback(const nav_msgs::Odometry::ConstPtr& pose);
  	void laserCallback(const sensor_msgs::LaserScan scanData);
	void stageSubjectPoseCallback(const nav_msgs::Odometry::ConstPtr& pose);
	void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& pose);
	void viconSubjectCallback(const geometry_msgs::TransformStamped::ConstPtr& pose);
	void clockCallback(const rosgraph_msgs::Clock clockData);
	void emergencyStopCallback(const std_msgs::Bool emergencyPauseData);
	void joyCallback(const sensor_msgs::Joy joyMessage);
	void setLights(int setting);

	/*for vicon*/
	geometry_msgs::TransformStamped latestPoses;
    	geometry_msgs::TransformStamped latestSubjectPoses;

	geometry_msgs::TransformStamped latestPosesAverage;
    	geometry_msgs::TransformStamped latestSubjectPosesAverage;

	std::vector<geometry_msgs::TransformStamped> latestPosesVector;
	std::vector<geometry_msgs::TransformStamped> latestSubjectPosesVector;

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

	ros::Subscriber joySub;
	
	ros::Publisher robot_pose_pub;

	ros::Publisher cmd_vel_pub;
	ros::Publisher multiplier_pub;
	
	ros::Subscriber clockSub;
	
	/*Publisher that says what sounds to play*/
	ros::Publisher audio_pub;

ros::Publisher goal_pub;
	ros::Publisher keyframe_pub;

	/*Movement orders for Pioneer*/
	geometry_msgs::Twist move_cmd;
	/*An audio request*/
    std_msgs::UInt16 audio_cmd;

	
		ros::Subscriber emergencyStopSub;

 	
public:
  Assert2Behaviour(ros::NodeHandle& nh);
  ~Assert2Behaviour();

  virtual void spin();
  virtual void spinOnce();

}; // class Assert2Behaviour

#endif // ASSERT2_BEHAVIOUR_H
