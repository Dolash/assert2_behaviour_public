#include <tf/transform_datatypes.h>
#include "assert2_behaviour/assert2_behaviour.h"

Assert2Behaviour::Assert2Behaviour(ros::NodeHandle& nh) 
    : nh(nh),
    privNh("~") {
		privNh.param<double>("loop_hz", loopHz, 10);
		privNh.param<bool>("stage_mode", stageMode, true);
		
		privNh.param<std::string>("pose_topic", poseTopic, "/robot1/pose");
		privNh.param<std::string>("subject_pose_topic", subjectPoseTopic, "/robot2/pose");
		privNh.param<std::string>("cmd_vel_topic", cmdVelTopic, "/cmd_vel");
		privNh.param<std::string>("scan_topic", scanTopic, "/scan");
		
		
		privNh.param<float>("start_x", startX, 1);
		privNh.param<float>("start_y", startY, 1);
		privNh.param<float>("end_x", goalX, 1);
		privNh.param<float>("end_y", goalY, 2);
		privNh.param<float>("start_yaw", startYaw, 0);
		privNh.param<float>("end_yaw", goalYaw, 0);
		privNh.param<float>("start_door_x", startDoorX, 1);
		privNh.param<float>("start_door_y", startDoorY, 1);
		privNh.param<float>("end_door_x", goalDoorX, 1);
		privNh.param<float>("end_door_y", goalDoorY, 2);
		privNh.param<float>("start_door_yaw", startDoorYaw, 0);
		privNh.param<float>("end_door_yaw", goalDoorYaw, 0);
		privNh.param<float>("door_x", doorX, 1);
		privNh.param<float>("door_y", doorY, 0);
		firstTime = false;
		poseReceived = false;
		subjectPoseReceived = false;

	/*state variables for which of the four movement goals to be targeting*/
	firstGoal = false;
        returnTrip = false;
	doorReached = true;

goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
		
		currentGoalX = goalDoorX;
		currentGoalY = goalDoorY;
		subjectDetected = false;
		clockReceived = false;
			emergencyPause.data = false;
		fighting = false;
        navigating = true;
laserReceived = false;
	multiplier.data = 1.0;
		
		//Set in both Stage and Vicon settings for a 5-second ROS wait before things begin
		timer = ros::Time::now();
		
		if (stageMode == false)
		{
			poseSub = nh.subscribe("/global_poses", 1, &Assert2Behaviour::viconCallback, this);
			subjectPoseSub = nh.subscribe("/vicon/subject/subject", 1, &Assert2Behaviour::viconSubjectCallback, this);
			
		}
		else
		{
			poseSub = nh.subscribe(poseTopic, 1, &Assert2Behaviour::stagePoseCallback, this);
			subjectPoseSub = nh.subscribe(subjectPoseTopic, 1, &Assert2Behaviour::stageSubjectPoseCallback, this);
			clockSub = nh.subscribe("/clock", 1, &Assert2Behaviour::clockCallback, this);
			
			
		}
		//Just here as an example
		//timer = clockTime.clock;
		emergencyStopSub = nh.subscribe("/emergency_stop", 1, &Assert2Behaviour::emergencyStopCallback, this);
		laserSub = nh.subscribe(scanTopic, 1, &Assert2Behaviour::laserCallback, this);
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmdVelTopic, 30);
		multiplier_pub = nh.advertise<std_msgs::Float32>(multiplierTopic, 30);
}

Assert2Behaviour::~Assert2Behaviour() {
 	ROS_INFO("[ASSERT2_BEHAVIOUR] Destroyed.");
}

void Assert2Behaviour::stagePoseCallback(const nav_msgs::Odometry::ConstPtr& pose) {
    stagePose = *pose;
	//ROS_INFO("[ASSERTIVE_BEHAVIOUR] stage pose.");
    poseReceived = true;
}

void Assert2Behaviour::stageSubjectPoseCallback(const nav_msgs::Odometry::ConstPtr& pose) {
    stageSubjectPose = *pose;
	//ROS_INFO("[ASSERTIVE_BEHAVIOUR] stage pose.");
    subjectPoseReceived = true;
}

void Assert2Behaviour::viconSubjectCallback(const geometry_msgs::TransformStamped::ConstPtr& pose) {

    latestSubjectPoses = *pose;
	subjectPoseReceived = true;
}
void Assert2Behaviour::viconCallback(const geometry_msgs::TransformStamped::ConstPtr& pose) {
    latestPoses = *pose;
    poseReceived = true;
}

void Assert2Behaviour::laserCallback(const sensor_msgs::LaserScan scanData) {
	//ROS_INFO("[ASSERTIVE_BEHAVIOUR] laser scan.");
    latestLaserScan = scanData.ranges;
    laserReceived = true;
}

void Assert2Behaviour::clockCallback(const rosgraph_msgs::Clock clockData)
{
	clockTime = clockData;
	//ROS_INFO("[ASSERTIVE_BEHAVIOUR] Clock.");
	clockReceived = true;
}

void Assert2Behaviour::emergencyStopCallback(const std_msgs::Bool emergencyPauseData)
{
	emergencyPause = emergencyPauseData;
	
}


/*For calculating the angle we want to be turned in order to be facing our target, given your pose and the goal location's pose*/
float Assert2Behaviour::getDesiredAngle(float targetX, float targetY, float currentXCoordinateIn, float currentYCoordinateIn)
{
	float result = 0;

	//I'm making this adjustment because I am turning the robot's current position into the origin, so the origin x and y are really currentPositionX/Y - currentPositionX/Y.
	//I'm doing this for my brain's sake.
	float nTargetX = (targetX - currentXCoordinateIn);
	float nTargetY = (targetY - currentYCoordinateIn);


	/*So this calculates, if our robot was at the origin and our target is at the appropriate relative position, what angle should we be turned to face them?*/
	float angbc = atan2((nTargetY), (nTargetX));
	result = angbc;
	result = (result - 1.57595);
	/*A quick fix in the event that this desired angle adjustment takes us "off the edge" of pi*/
	if (result < -3.1415)
    {
		//result = (3.1518 - (fabs(result) - 3.1518));
		result += 6.2831;
	}
	else if (result > 3.1415)
	{
	    result -= 6.2831;
	}

	
	return result;
}


void Assert2Behaviour::subjectAhead()
{
	float currentX;
	float currentY;
	float subjectCurrentX;
	float subjectCurrentY;
	float yaw;
	float angleToSubject;



	if (stageMode == true)
	{
		currentX = stagePose.pose.pose.position.x;
		currentY = stagePose.pose.pose.position.y;
		subjectCurrentX = stageSubjectPose.pose.pose.position.x;
		subjectCurrentY = stageSubjectPose.pose.pose.position.y;
		yaw = tf::getYaw(stagePose.pose.pose.orientation);
	}
	else
	{
		currentX = latestPoses.transform.translation.x;
		currentY = latestPoses.transform.translation.y;
		subjectCurrentX = latestSubjectPoses.transform.translation.x;
		subjectCurrentY = latestSubjectPoses.transform.translation.y;
		yaw = tf::getYaw(latestPoses.transform.rotation);;
	}
	angleToSubject = getDesiredAngle(subjectCurrentX, subjectCurrentY, currentX, currentY);
    
    /*Take your pose and orientation compare to the pose of every other tracked subject and if one is detected then set that detection to true*/
    /*if (fabs(currentX - subjectCurrentX) < 1.0 
		&& fabs(currentY - subjectCurrentY) < 1.0 
		&& ((yaw > (angleToSubject - 0.25)) && (yaw < (angleToSubject + 0.25)) || (((yaw > 2.9) && (angleToSubject < -2.9)) 
		|| ((yaw < -2.9) && (angleToSubject > 2.9)))))*/

if(
sqrt(((doorX - currentX)*(doorX - currentX)) + ((doorY - currentY)*(doorY - currentY))) < 1.5
	&&	
sqrt(((doorX - subjectCurrentX)*(doorX - subjectCurrentX)) + ((doorY - subjectCurrentY)*(doorY - subjectCurrentY))) < 1.5
	&&
sqrt(((currentGoalX - subjectCurrentX)*(currentGoalX - subjectCurrentX)) + ((currentGoalY - subjectCurrentY)*(currentGoalY - subjectCurrentY))) < sqrt(((currentGoalX - currentX)*(currentGoalX - currentX)) + ((currentGoalY - currentY)*(currentGoalY - currentY)))
 	)
    {
        //ROS_INFO("[ASSERTIVE_BEHAVIOUR] SUBJECT AHEAD");
        subjectDetected = true;
    }
    else 
    {
        subjectDetected = false;
    }



	if (subjectDetected == true)
		{
			if (
	sqrt(((doorX - currentX)*(doorX - currentX)) + ((doorY - currentY)*(doorY - currentY))) 
		<	
	sqrt(((doorX - subjectCurrentX)*(doorX - subjectCurrentX)) + ((doorY - subjectCurrentY)*(doorY - subjectCurrentY)))
		)
			{
				multiplier.data = 1.25;
				multiplier_pub.publish(multiplier);
						
			}
			else
			{
				multiplier.data = 0.5;
				multiplier_pub.publish(multiplier);
			
			}
		}
		else
		{
			multiplier.data = 1.0;
			multiplier_pub.publish(multiplier);
		}



}


void Assert2Behaviour::fightingBehaviour()
{
 

}

void Assert2Behaviour::navigatingBehaviour()
{
	ros::Time tempTime;
	if (stageMode == false)
	{
		tempTime = ros::Time::now();
	}
	else
	{
		tempTime = clockTime.clock;
	}
	subjectAhead();



	
	waypointing();
	

	/*
	if (subjectDetected == true  && brave == false)
	{
		navigating = false;
            //move_cmd.linear.x = -0.35;
		move_cmd.linear.x = 0.0;
            move_cmd.angular.z = 0.0;
            cmd_vel_pub.publish(move_cmd);            
            fighting = true;
            timer = tempTime;
            //moveOrderTimer = tempTime;
            setLights(fightStartLights);
    }
    else if (subjectDetected == true  && brave == true) 
    {*/
		
		
		//if(latestSonarScan[3] < 0.5 || latestSonarScan[4] < 0.5)
		/*for(int i = 20; i < scrubbedScan.ranges.size() - 20; i++)
		{
			float allowedDistanceScrubbed = 0.4 + (0.20 - (0.20*(fabs(90 - i)/90)));
			//float allowedDistanceInverse = 0.15 + (0.15 - (0.15*(fabs(90 - i)/90)));
			if (scrubbedScan.ranges[i] < allowedDistanceScrubbed && scrubbedScan.ranges[i] != 0.00)
			{
				//ROS_INFO("[ASSERTIVE_BEHAVIOUR]In my wayyyyyyyyyyy");
				emergencyPause = true;
			}
		}*/
		
		/*
		if (emergencyPause.data == true)
		{
			
			
			if (timer + ros::Duration(10 - aggression) < tempTime)
			{
				
				brave = false;
				navigating = false;
				audio_cmd.data = loseFightSound;
                		audio_pub.publish(audio_cmd);
                		setLights(loseFightLights);
                		timer = tempTime;
                		defeat = true;
				fighting = true;
                		fightStarting = false;
				//TODO: Minister Demo "non-moving" fix
				//angleAtDefeat = tf::getYaw(amclPose.pose.pose.orientation);
				fightSoundPlayed = false;
				winner.data = false;
				winner_pub.publish(winner);

			}
			else
			{
				move_cmd.linear.x = 0.0;
            			move_cmd.angular.z = 0.0;
            			cmd_vel_pub.publish(move_cmd); 
			}
		}
		else
		{
                	timer = tempTime;
			//TODO: Minister Demo "non-moving" fix
                	//waypointing();
		}
                //ROS_INFO("[ASSERTIVE_BEHAVIOUR] WINNING");
        }
        else if (subjectDetected == false  && brave == true) 
        {
            if (timer + ros::Duration(2) < tempTime) 
            {
               // ROS_INFO("[ASSERTIVE_BEHAVIOUR] WON");
                audio_cmd.data = backToNormalSound;
                audio_pub.publish(audio_cmd);
                setLights(backToNormalLights);
                brave = false;
				winner.data = false;
				winner_pub.publish(winner);
            }
			waypointing();
		} 
        else
        {
            ROS_INFO("[ASSERTIVE_BEHAVIOUR] Waypointing");
			waypointing();
        }*/
        
   
}


void Assert2Behaviour::waypointing()
{
	
	float desiredX;
	float desiredY;
	float desiredAngle = 0;
	float yaw;		
			
	
	geometry_msgs::Quaternion currentOrientation;
	float currentX;
	float currentY;
	
	
		if (stageMode == true)
			{
				currentOrientation = stagePose.pose.pose.orientation;
				currentX = stagePose.pose.pose.position.x;
				currentY = stagePose.pose.pose.position.y;
			}
			else
			{
				currentOrientation = latestPoses.transform.rotation;
				currentX = latestPoses.transform.translation.x;
				currentY = latestPoses.transform.translation.y;
			}
	
			/*From the starting point to the end point, but after the special start condition*/
			if (returnTrip == false && firstGoal == true)
			{
				/*Hadn't reached the point just through the door yet, meaning you are now, and will now drive to the end point*/
				if (doorReached == false)
				{
					float yawDiff = tf::getYaw(currentOrientation) - goalDoorYaw;
					if (yawDiff > 3.14159)
					{
						yawDiff += -6.2831;
				    	}
					else if (yawDiff < -3.14159)
					{
						yawDiff += 6.2831;
					}
					if (((fabs(currentX - goalDoorX) < 0.4 && fabs(currentY - goalDoorY) < 0.4) ))//&& (fabs(yawDiff) < 0.3)
					{
							doorReached = true;
							currentGoalX = goalX;
							currentGoalY = goalY;

							std_msgs::Header tmpHead;
							geometry_msgs::Pose tmpPose;
							geometry_msgs::Point tmpPoint;
							geometry_msgs::Quaternion tmpQuaternion;
							tmpPoint.x = goalX;
							tmpPoint.y = goalY;
							tmpPoint.z = 0.0;
							tmpQuaternion = tf::createQuaternionMsgFromYaw(goalYaw);
							tmpPose.position = tmpPoint;
							tmpPose.orientation = tmpQuaternion;
							goal_cmd.pose =	tmpPose;
							tmpHead.frame_id = "map";
							goal_cmd.header = tmpHead;

							goal_pub.publish(goal_cmd);
							

					}
				}
				/*Have now reached the end point and will turn around to drive back to the door facing the start point, aiming at a point just through the door*/
				else
				{
			
					float yawDiff = tf::getYaw(currentOrientation) - goalYaw;
					if (yawDiff > 3.14159)
					{
						yawDiff += -6.2831;
				    	}
					else if (yawDiff < -3.14159)
					{
						yawDiff += 6.2831;
					}
					if (((fabs(currentX - goalX) < 0.4 && fabs(currentY - goalY) < 0.4) ))//&& (fabs(yawDiff) < 0.3)
					{
							returnTrip = true;
							doorReached = false;
							currentGoalX = startDoorX;
							currentGoalY = startDoorY;

							std_msgs::Header tmpHead;
							geometry_msgs::Pose tmpPose;
							geometry_msgs::Point tmpPoint;
							geometry_msgs::Quaternion tmpQuaternion;
							tmpPoint.x = startDoorX;
							tmpPoint.y = startDoorY;
							tmpPoint.z = 0.0;
							tmpQuaternion = tf::createQuaternionMsgFromYaw(startDoorYaw);
							tmpPose.position = tmpPoint;
							tmpPose.orientation = tmpQuaternion;
							goal_cmd.pose =	tmpPose;
							tmpHead.frame_id = "map";
							goal_cmd.header = tmpHead;

							goal_pub.publish(goal_cmd);

						
					}
				}
			}
			/*Are on the return trip or the special first round*/
			else
			{
				/*On the way back from the end point and hadn't reached the point just through the door yet, but will now, and will now aim for the start point*/
				if (doorReached == false)
				{
					float yawDiff = tf::getYaw(currentOrientation) - startDoorYaw;
					if (yawDiff > 3.14159)
					{
						yawDiff += -6.2831;
				    	}
					else if (yawDiff < -3.14159)
					{
						yawDiff += 6.2831;
					}
					if ((fabs(currentX - startDoorX) < 0.4 && fabs(currentY - startDoorY) < 0.4) ) //&&  (fabs(yawDiff) < 0.3)
					{
							doorReached = true;
							currentGoalX = startX;
							currentGoalY = startY;
						
							std_msgs::Header tmpHead;
							geometry_msgs::Pose tmpPose;
							geometry_msgs::Point tmpPoint;
							geometry_msgs::Quaternion tmpQuaternion;
							tmpPoint.x = startX;
							tmpPoint.y = startY;
							tmpPoint.z = 0.0;
							tmpPose.position = tmpPoint;
							tmpQuaternion = tf::createQuaternionMsgFromYaw(startYaw);
							tmpPose.orientation = tmpQuaternion;
							goal_cmd.pose =	tmpPose;
							tmpHead.frame_id = "map";
							goal_cmd.header = tmpHead;

							goal_pub.publish(goal_cmd);
							
						
					}
				}
				else
				/*On the return trip and have reached the point near the other side of the door, now looking for when you arrive at the start to re-target the goal door*/
				{
					ROS_INFO("[ASSERTIVE_BEHAVIOUR] Am I in the right place?");
					float yawDiff = tf::getYaw(currentOrientation) - startYaw;
					if (yawDiff > 3.14159)
					{
						yawDiff += -6.2831;
				    	}
					else if (yawDiff < -3.14159)
					{
						yawDiff += 6.2831;
					}
					if ((fabs(currentX - startX) < 0.4 && fabs(currentY - startY) < 0.4)) // &&  (fabs(yawDiff) < 0.3)
					{
							ROS_INFO("[ASSERTIVE_BEHAVIOUR] Goal sent!");
							firstGoal = true;
							returnTrip = false;
							doorReached = false;
							currentGoalX = goalDoorX;
							currentGoalY = goalDoorY;


							std_msgs::Header tmpHead;
							geometry_msgs::Pose tmpPose;
							geometry_msgs::Point tmpPoint;
							geometry_msgs::Quaternion tmpQuaternion;
							tmpPoint.x = goalDoorX;
							tmpPoint.y = goalDoorY;
							tmpPoint.z = 0.0;
							tmpPose.position = tmpPoint;
							tmpQuaternion = tf::createQuaternionMsgFromYaw(goalDoorYaw);
							tmpPose.orientation = tmpQuaternion;
							goal_cmd.pose =	tmpPose;
							tmpHead.frame_id = "map";
							goal_cmd.header = tmpHead;

							goal_pub.publish(goal_cmd);

						
					}
				}
			}
		
		
}



void Assert2Behaviour::spinOnce() {
	ros::Rate rate(loopHz);
	
	if (laserReceived == true && poseReceived == true && subjectPoseReceived == true)

    {
		
	    if((timer + ros::Duration(5) < ros::Time::now()) && firstTime == false)
	    {

			firstTime = true;
			ROS_INFO("[ASSERTIVE_BEHAVIOUR] Let's get started!");
	    } 
	     
	    else if (firstTime == true)
		{    
			if(fighting)
			{
				fightingBehaviour();
			}
			else if(navigating)
			{
				
				navigatingBehaviour();

			}
			else
			{
				ROS_INFO("[ASSERTIVE_BEHAVIOUR] Stateless");
			}
	
		}
		else
		{
			/*waiting a few seconds to make sure everything else started up and is ready to go*/
		}
	  	
	}
	else
	{
		ROS_INFO("[ASSERTIVE_BEHAVIOUR] Stage mode: %d, pose received: %d, subject pose received: %d", stageMode, poseReceived, subjectPoseReceived);
	}
	
	rate.sleep();
	ros::spinOnce();
	
	
}

/*Gets called by main, runs all the time at the given rate.*/
void Assert2Behaviour::spin() {
  //ros::Rate rate(loopHz);
  while (ros::ok()) {
    spinOnce();
	//rate.sleep();
  }
}