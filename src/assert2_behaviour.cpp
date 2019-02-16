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
		privNh.param<std::string>("multiplier_topic", multiplierTopic, "/multiplier");
	
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

		privNh.param<float>("park_diff_x", parkDiffX, 1.0);
		privNh.param<float>("park_diff_y", parkDiffY, 0.75);

		privNh.param<float>("door_engage_distance", doorEngageDistance, 1.5);
		privNh.param<float>("subject_door_engage_distance", subjectDoorEngageDistance, 1.5);
		privNh.param<float>("winner_multiple", winnerMultiple, 1.25);
		privNh.param<bool>("one_run", oneRun, false);
		privNh.param<std::string>("map_frame", mapFrame, "map");

		privNh.param<std::string>("joy_topic", joyTopic, "/joy");

		firstTime = false;
		poseReceived = false;
		subjectPoseReceived = false;

		listeningForUnpause = false;
		unpaused = false;
		bool clearLights;

		/*state variables for which of the four movement goals to be targeting*/
		firstGoal = false;
		returnTrip = false;
		doorReached = true;

		clearedA = false;
		clearedB = false;
		fightStart = false;

		parkX = 0.0;
		parkY = 0.0;

		subjectDetected = false;
		clockReceived = false;
		emergencyPause.data = false;
		fighting = false;
        	navigating = true;
		laserReceived = true;
		
		currentGoalX = goalDoorX;
		currentGoalY = goalDoorY;
		currentGoalOrientation = tf::createQuaternionMsgFromYaw(goalDoorYaw);
		
		multiplier.data = 1.0;
		currentVelocity = 1.0;
		currentSubjectVelocity = 1.0;

		startupLights = 0;
        	fightStartLights = 1;
        	loseFightLights = 2;
        	winFightLights = 3;
        	backToNormalLights = 4;

		startupSound = 22;
        	fightStartSound = 23;
        	loseFightSound = 24;
        	winFightSound = 25;
        	backToNormalSound = 26;

		tempColor.r = 0;
        	tempColor.g = 0;
        	tempColor.b = 0;
        	tempColor.a = 0;
        	for (int i = 0; i < 1; i++)
        	{
            		lightColours.push_back(tempColor);
        	}
        	lights.color_pattern = lightColours;
		
		//Set in both Stage and Vicon settings for a 5-second ROS wait before things begin
		timer = ros::Time::now();
		//Just here as an example
		//timer = clockTime.clock;

		if (stageMode == false)
		{
			poseSub = nh.subscribe(poseTopic, 1, &Assert2Behaviour::viconCallback, this);
			subjectPoseSub = nh.subscribe(subjectPoseTopic, 1, &Assert2Behaviour::viconSubjectCallback, this);
			
		}
		else
		{
			poseSub = nh.subscribe(poseTopic, 1, &Assert2Behaviour::stagePoseCallback, this);
			subjectPoseSub = nh.subscribe(subjectPoseTopic, 1, &Assert2Behaviour::stageSubjectPoseCallback, this);
			clockSub = nh.subscribe("/clock", 1, &Assert2Behaviour::clockCallback, this);
			
			
		}
		joySub = nh.subscribe(joyTopic, 1, &Assert2Behaviour::joyCallback, this);
		emergencyStopSub = nh.subscribe("/emergency_stop", 1, &Assert2Behaviour::emergencyStopCallback, this);
		laserSub = nh.subscribe(scanTopic, 1, &Assert2Behaviour::laserCallback, this);
		goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmdVelTopic, 30);
		multiplier_pub = nh.advertise<std_msgs::Float32>(multiplierTopic, 30);
		keyframe_pub = nh.advertise<autonomy_leds_msgs::Keyframe>("/leds/display", 1);
		audio_pub = nh.advertise<std_msgs::UInt16>("/sound_player", 1, true);
		robot_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/roy_robot_pose", 1, true);
}

Assert2Behaviour::~Assert2Behaviour() {
 	ROS_INFO("[ASSERT2_BEHAVIOUR] Destroyed.");
}

void Assert2Behaviour::stagePoseCallback(const nav_msgs::Odometry::ConstPtr& pose) {
	stagePose = *pose;
	poseReceived = true;
}

void Assert2Behaviour::stageSubjectPoseCallback(const nav_msgs::Odometry::ConstPtr& pose) {
	stageSubjectPose = *pose;
	subjectPoseReceived = true;
}

void Assert2Behaviour::viconSubjectCallback(const geometry_msgs::TransformStamped::ConstPtr& pose) {
	latestSubjectPoses = *pose;
	subjectPoseReceived = true;
}
void Assert2Behaviour::viconCallback(const geometry_msgs::TransformStamped::ConstPtr& pose) {
	latestPoses = *pose;
	latestPosesVector.insert(latestPosesVector.end(),latestPoses);
	if(latestPosesVector.size() > 7)
	{
		latestPosesVector.erase(latestPosesVector.begin());
	}
			std_msgs::Header tmpHead;
			geometry_msgs::Pose tmpPose;
			geometry_msgs::Point tmpPoint;
			//geometry_msgs::Quaternion tmpQuaternion;
			tmpPoint.x = latestPoses.transform.translation.x;
			tmpPoint.y = latestPoses.transform.translation.y;
			tmpPoint.z = 0.0;

			tmpPose.position = tmpPoint;
			tmpPose.orientation = latestPoses.transform.rotation;
			//tmpPose.orientation = tmpQuaternion;
			robotPose.pose = tmpPose;
			tmpHead.frame_id = "world";
			robotPose.header = tmpHead;
			robot_pose_pub.publish(robotPose);
	poseReceived = true;
}

void Assert2Behaviour::laserCallback(const sensor_msgs::LaserScan scanData) {
	latestLaserScan = scanData.ranges;
	laserReceived = true;
}

void Assert2Behaviour::clockCallback(const rosgraph_msgs::Clock clockData)
{
	clockTime = clockData;
	clockReceived = true;
}

void Assert2Behaviour::emergencyStopCallback(const std_msgs::Bool emergencyPauseData)
{
	emergencyPause = emergencyPauseData;
}



void Assert2Behaviour::setLights(int setting)
{

    if (setting == 0)
    {
        lights.duration = 60;
        lights.start_index = 12;
        lights.pattern_repeat = 10;
        for (int i = 0; i < 1; i++)
        {
            lights.color_pattern[i].r = 1;
            lights.color_pattern[i].g = 1;
            lights.color_pattern[i].b = 1;
            lights.color_pattern[i].a = 0;
        }
        
    }
    else if (setting == 1)
    {
        lights.duration = 60;
        lights.start_index = 12;
        lights.pattern_repeat = 10;
        for (int i = 0; i < 1; i++)
        {
            lights.color_pattern[i].r = 0;
            lights.color_pattern[i].g = 1;
            lights.color_pattern[i].b = 1;
            lights.color_pattern[i].a = 0;
        }
    }
    else if (setting == 2)
    {
        lights.duration = 60;
        lights.start_index = 12;
        lights.pattern_repeat = 10;
        for (int i = 0; i < 1; i++)
        {
            lights.color_pattern[i].r = 1.0;
            lights.color_pattern[i].g = 0;
            lights.color_pattern[i].b = 0;
            lights.color_pattern[i].a = 0;
        }
    }
    else if (setting == 3)
    {
        lights.duration = 60;
        lights.start_index = 12;
        lights.pattern_repeat = 10;
        for (int i = 0; i < 1; i++)
        {
            lights.color_pattern[i].r = 0;
            lights.color_pattern[i].g = 1;
            lights.color_pattern[i].b = 0;
            lights.color_pattern[i].a = 0;
        }
    }
    else if (setting == 4)
    {
        lights.duration = 60;
        lights.start_index = 12;
        lights.pattern_repeat = 10;
        for (int i = 0; i < 1; i++)
        {
		
            		lights.color_pattern[i].r = 0;
            		lights.color_pattern[i].g = 0;
            		lights.color_pattern[i].b = 1;
            		lights.color_pattern[i].a = 0;
		
		
        }
    }
	

    keyframe_pub.publish(lights);
}


void Assert2Behaviour::joyCallback(const sensor_msgs::Joy joyMessage) {
	//ROS_INFO("[ASSERTIVE_BEHAVIOUR] Joy message received.");
	if (listeningForUnpause == true)
	{
		ROS_INFO("[ASSERTIVE_BEHAVIOUR] listening.");
		if (joyMessage.buttons[1] == 1)
		{
			unpaused = true;
			ROS_INFO("[ASSERTIVE_BEHAVIOUR] Unpaused.");
		}
		else
		{
			ROS_INFO("[ASSERTIVE_BEHAVIOUR] Wrong button pressed or no button pressed.");
		}
	}
	else
	{
		ROS_INFO("[ASSERTIVE_BEHAVIOUR] Not listening.");
	}
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
	//result = (result - 1.57595);
	/*A quick fix in the event that this desired angle adjustment takes us "off the edge" of pi*/
	if (result < -3.1415)
	{
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
		yaw = tf::getYaw(latestPoses.transform.rotation);
	}
	angleToSubject = getDesiredAngle(subjectCurrentX, subjectCurrentY, currentX, currentY);
    

//I don't remember what this chunk does but I won't delete it just in case
    /*Take your pose and orientation compare to the pose of every other tracked subject and if one is detected then set that detection to true*/
    /*if (fabs(currentX - subjectCurrentX) < 1.0 
		&& fabs(currentY - subjectCurrentY) < 1.0 
		&& ((yaw > (angleToSubject - 0.25)) && (yaw < (angleToSubject + 0.25)) || (((yaw > 2.9) && (angleToSubject < -2.9)) 
		|| ((yaw < -2.9) && (angleToSubject > 2.9)))))*/



/*The below is saying: If you're within 1.5 of the door, and your subject is within 1.5 of the door, and your goal is currently closer to your subject than to you (meaning that it lies somewhere on the other side of the door), which are the conditions for conflict*/
	if(
	sqrt(((doorX - currentX)*(doorX - currentX)) + ((doorY - currentY)*(doorY - currentY))) < doorEngageDistance
	&&	
	sqrt(((doorX - subjectCurrentX)*(doorX - subjectCurrentX)) + ((doorY - subjectCurrentY)*(doorY - subjectCurrentY))) < subjectDoorEngageDistance
	&&
	//sqrt(((currentGoalX - subjectCurrentX)*(currentGoalX - subjectCurrentX)) + ((currentGoalY - subjectCurrentY)*(currentGoalY - subjectCurrentY))) < sqrt(((currentGoalX - currentX)*(currentGoalX - currentX)) + ((currentGoalY - currentY)*(currentGoalY - currentY)))
	//((currentY < doorY && currentY < subjectCurrentY) || (currentY > doorY && currentY > subjectCurrentY))
	((returnTrip == false && currentY < subjectCurrentY) || (returnTrip == true && currentY > subjectCurrentY))
 	)
	{
        
		subjectDetected = true;
	}
	else 
	{
        	subjectDetected = false;
	}



	if (subjectDetected == true)
	{

			/*If you are closer to the door than your opponant, carry on, speed up*/
		if (
		(sqrt(((doorX - currentX)*(doorX - currentX)) + ((doorY - currentY)*(doorY - currentY))) )
		<	
		(sqrt(((doorX - subjectCurrentX)*(doorX - subjectCurrentX)) + ((doorY - subjectCurrentY)*(doorY - subjectCurrentY))) )
		)
		{
			multiplier.data = winnerMultiple;
			multiplier_pub.publish(multiplier);
			if (fightStart == false)
			{
				
				audio_cmd.data = winFightSound;
                		audio_pub.publish(audio_cmd);
                		setLights(winFightLights);
				fightStart = true;
			}
						
		}
			/*If you are further than your opponant, go into park*/
		else
		{
			navigating = false;
			fighting = true;
				
			audio_cmd.data = loseFightSound;
                	audio_pub.publish(audio_cmd);
                	setLights(loseFightLights);

			if (currentX < subjectCurrentX)
			{
				parkX = doorX - parkDiffX;
			}
			else
			{
				parkX = doorX + parkDiffX;
			}
			if (currentY < doorY)
			{
				parkY = doorY - parkDiffY;
			}
			else
			{
				parkY = doorY + parkDiffY;
			}
			std_msgs::Header tmpHead;
			geometry_msgs::Pose tmpPose;
			geometry_msgs::Point tmpPoint;
			geometry_msgs::Quaternion tmpQuaternion;
			tmpPoint.x = parkX;
			tmpPoint.y = parkY;
			tmpPoint.z = 0.0;

			/*tf::Quaternion q_rot, q_orig, q_new;
			double r=0, p=0, y=0.7854;
			q_rot = tf::createQuaternionFromRPY(r, p, y);
			quaternionMsgToTF(currentGoalOrientation, q_orig);
			q_new = q_rot*q_orig;
			q_new.normalize();
			quaternionTFToMsg(q_new, tmpQuaternion);*/ 
			tmpPose.position = tmpPoint;
			if (returnTrip == false)
			{
				tmpQuaternion = tf::createQuaternionMsgFromYaw(startYaw);
			}
			else
			{
				tmpQuaternion = tf::createQuaternionMsgFromYaw(goalYaw);
			}
			tmpPose.orientation = tmpQuaternion;
			//tmpPose.orientation = tmpQuaternion;
			goal_cmd.pose =	tmpPose;
			tmpHead.frame_id = mapFrame;
			goal_cmd.header = tmpHead;
			
			multiplier.data = 1.0;
			multiplier_pub.publish(multiplier);
			goal_pub.publish(goal_cmd);
			ROS_INFO("[ASSERTIVE_BEHAVIOUR] PARKING");
			
		
		}
	}
	else
	{
		
		multiplier.data = 1.0;
		multiplier_pub.publish(multiplier);
		fightStart = false;
	}



}


void Assert2Behaviour::fightingBehaviour()
{
	float currentX;
	float currentY;
	float subjectCurrentX;
	float subjectCurrentY;
	geometry_msgs::Quaternion currentOrientation;


	if (stageMode == true)
	{
		currentX = stagePose.pose.pose.position.x;
		currentY = stagePose.pose.pose.position.y;
		subjectCurrentX = stageSubjectPose.pose.pose.position.x;
		subjectCurrentY = stageSubjectPose.pose.pose.position.y;
	}
	else
	{
		currentX = latestPoses.transform.translation.x;
		currentY = latestPoses.transform.translation.y;
		subjectCurrentX = latestSubjectPoses.transform.translation.x;
		subjectCurrentY = latestSubjectPoses.transform.translation.y;
		currentOrientation = latestPoses.transform.rotation;
	}


	float yawDiff;
	if (returnTrip == false)
	{
		if (clearedA == true && clearedB == false)
		{
			
			yawDiff = tf::getYaw(currentOrientation) - desiredYaw;
		}
		else
		{
			yawDiff = tf::getYaw(currentOrientation) - goalDoorYaw;
		}
		
		if (yawDiff > 3.14159)
		{
			yawDiff += -6.2831;
	    	}
		else if (yawDiff < -3.14159)
		{
			yawDiff += 6.2831;
		}
	}
	else
	{
		if (clearedA == true && clearedB == false)
		{
			
			yawDiff = tf::getYaw(currentOrientation) - desiredYaw;
		}
		else
		{
			yawDiff = tf::getYaw(currentOrientation) - startDoorYaw;
		}
		if (yawDiff > 3.14159)
		{
			yawDiff += -6.2831;
	    	}
		else if (yawDiff < -3.14159)
		{
			yawDiff += 6.2831;
		}
	}
			


	/*If the subject is now far enough away from the door that, by the previous term used to start the fight, they must now be moving away from the conflict*/	
	//if (sqrt(((doorX - subjectCurrentX)*(doorX - subjectCurrentX)) + ((doorY - subjectCurrentY)*(doorY - subjectCurrentY))) > (subjectDoorEngageDistance + 0.1) && clearedA == false)
	if ( clearedA == false && clearedB == false
		&&
		(
		(((fabs(currentX - parkX) < 0.4 && fabs(currentY - parkY) < 0.4) ) && (fabs(yawDiff) < 0.3)) 
		||
		(sqrt(((doorX - subjectCurrentX)*(doorX - subjectCurrentX)) + ((doorY - subjectCurrentY)*(doorY - subjectCurrentY))) > (sqrt(((doorX - currentX)*(doorX - currentX)) + ((doorY - currentY)*(doorY - currentY))) + 0.15))
		)
		
		)
	{
		std_msgs::Header tmpHead;
		geometry_msgs::Pose tmpPose;
		geometry_msgs::Point tmpPoint;
		geometry_msgs::Quaternion tmpQuaternion;
		tmpPoint.x = currentX;
		tmpPoint.y = currentY;
		tmpPoint.z = 0.0;		
		if (returnTrip == false)
		{
			desiredYaw = getDesiredAngle(goalDoorX, goalDoorY, currentX, currentY);
		}
		else
		{
			desiredYaw = getDesiredAngle(startDoorX, startDoorY, currentX, currentY);
		}
		tmpQuaternion = tf::createQuaternionMsgFromYaw(desiredYaw);
		tmpPose.position = tmpPoint;
		tmpPose.orientation = tmpQuaternion;
		goal_cmd.pose =	tmpPose;
		tmpHead.frame_id = mapFrame;
		goal_cmd.header = tmpHead;

		goal_pub.publish(goal_cmd);
		ROS_INFO("[ASSERTIVE_BEHAVIOUR] Turning");
		audio_cmd.data = backToNormalSound;
		audio_pub.publish(audio_cmd);
		setLights(backToNormalLights);

		clearedA = true;
		
	}
	else if (clearedA == true && clearedB == false
		&&
		/*now turned toward door point*/	
		(fabs(yawDiff) < 0.3)
		&&
		sqrt(((doorX - subjectCurrentX)*(doorX - subjectCurrentX)) + ((doorY - subjectCurrentY)*(doorY - subjectCurrentY))) > (sqrt(((doorX - currentX)*(doorX - currentX)) + ((doorY - currentY)*(doorY - currentY))) + 0.15
		)
		)
	{
		std_msgs::Header tmpHead;
		geometry_msgs::Pose tmpPose;
		geometry_msgs::Point tmpPoint;
		geometry_msgs::Quaternion tmpQuaternion;
		if (returnTrip == false)
		{
			tmpPoint.x = goalDoorX;
			tmpPoint.y = goalDoorY;
			tmpPoint.z = 0.0;
			tmpQuaternion = tf::createQuaternionMsgFromYaw(goalDoorYaw);
		}
		else
		{
			tmpPoint.x = startDoorX;
			tmpPoint.y = startDoorY;
			tmpPoint.z = 0.0;
			
			tmpQuaternion = tf::createQuaternionMsgFromYaw(startDoorYaw);
		}
		
		tmpPose.position = tmpPoint;
		tmpPose.orientation = tmpQuaternion;
		goal_cmd.pose =	tmpPose;
		tmpHead.frame_id = mapFrame;
		goal_cmd.header = tmpHead;

		goal_pub.publish(goal_cmd);

		clearedB = true;
		
		
	}
	else if(clearedA == true && clearedB == true
		&&
		( (((fabs(currentX - goalDoorX) < 0.4 && fabs(currentY - goalDoorY) < 0.4) ) && (fabs(yawDiff) < 0.3)) || (((fabs(currentX - startDoorX) < 0.4 && fabs(currentY - startDoorY) < 0.4) ) && (fabs(yawDiff) < 0.3)))
		)
	{
		std_msgs::Header tmpHead;
		geometry_msgs::Pose tmpPose;
		geometry_msgs::Point tmpPoint;
		geometry_msgs::Quaternion tmpQuaternion;
		tmpPoint.x = currentGoalX;
		tmpPoint.y = currentGoalY;
		tmpPoint.z = 0.0;
		tmpQuaternion = currentGoalOrientation;
		
		tmpPose.position = tmpPoint;
		tmpPose.orientation = tmpQuaternion;
		goal_cmd.pose =	tmpPose;
		tmpHead.frame_id = mapFrame;
		goal_cmd.header = tmpHead;

		goal_pub.publish(goal_cmd);

		clearedA = false;
		clearedB = false;
		ROS_INFO("[ASSERTIVE_BEHAVIOUR] Returning");
		navigating = true;
		fighting = false;
	}
	
 	/*else if (((fabs(currentX - parkX) < 0.4 && fabs(currentY - parkY) < 0.4))) //) && clearedB == false ) //&& (fabs(yawPark) < 0.3)
	{
		std_msgs::Header tmpHead;
		geometry_msgs::Pose tmpPose;
		geometry_msgs::Point tmpPoint;
		geometry_msgs::Quaternion tmpQuaternion;
		//tmpPoint.x = doorX;
		//tmpPoint.y = doorY;
		tmpPoint.x = currentGoalX;
		tmpPoint.y = currentGoalY;
		tmpPoint.z = 0.0;
		tmpQuaternion = currentGoalOrientation;
		tmpPose.position = tmpPoint;
		tmpPose.orientation = tmpQuaternion;
		goal_cmd.pose =	tmpPose;
		tmpHead.frame_id = mapFrame;
		goal_cmd.header = tmpHead;

		goal_pub.publish(goal_cmd);
		ROS_INFO("[ASSERTIVE_BEHAVIOUR] TURNING");
		audio_cmd.data = backToNormalSound;
		audio_pub.publish(audio_cmd);
		setLights(backToNormalLights);

		clearedB = true;
	}*/
	/* to remind you, it was if / if / else if before
	else if (clearedA == true && clearedB == true)
	{
		std_msgs::Header tmpHead;
		geometry_msgs::Pose tmpPose;
		geometry_msgs::Point tmpPoint;
		geometry_msgs::Quaternion tmpQuaternion;
		tmpPoint.x = currentGoalX;
		tmpPoint.y = currentGoalY;
		tmpPoint.z = 0.0;
		tmpQuaternion = currentGoalOrientation;
		tmpPose.position = tmpPoint;
		tmpPose.orientation = tmpQuaternion;
		goal_cmd.pose =	tmpPose;
		tmpHead.frame_id = mapFrame;
		goal_cmd.header = tmpHead;

		goal_pub.publish(goal_cmd);
		ROS_INFO("[ASSERTIVE_BEHAVIOUR] RESUMING");
		
		clearedA = false;
		clearedB = false;
		navigating = true;
		fighting = false;

		

	}*/


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
	//ROS_INFO("[ASSERTIVE_BEHAVIOUR] Do I make it to nav?");

	subjectAhead();

	//ROS_INFO("[ASSERTIVE_BEHAVIOUR] Done ahead");
	
	
	waypointing();
	//ROS_INFO("[ASSERTIVE_BEHAVIOUR] waypointed");
	//Below is a bunch of stuff left over from the previous version of the system that I can pick at if need be.


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
			if (((fabs(currentX - goalDoorX) < 0.4 && fabs(currentY - goalDoorY) < 0.4) ) && (fabs(yawDiff) < 0.3))
			{
				doorReached = true;
				currentGoalX = goalX;
				currentGoalY = goalY;
				currentGoalOrientation = tf::createQuaternionMsgFromYaw(goalYaw);
				std_msgs::Header tmpHead;
				geometry_msgs::Pose tmpPose;
				geometry_msgs::Point tmpPoint;
				geometry_msgs::Quaternion tmpQuaternion;
				tmpPoint.x = goalX;
				tmpPoint.y = goalY;
				tmpPoint.z = 0.0;
				tmpQuaternion = currentGoalOrientation;
				tmpPose.position = tmpPoint;
				tmpPose.orientation = tmpQuaternion;
				goal_cmd.pose =	tmpPose;
				tmpHead.frame_id = mapFrame;
				goal_cmd.header = tmpHead;
				//ROS_INFO("[ASSERTIVE_BEHAVIOUR] LOSING");
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
			if (((fabs(currentX - goalX) < 0.4 && fabs(currentY - goalY) < 0.4) ) && (fabs(yawDiff) < 0.3))
			{
				if(unpaused == false)
				{

					listeningForUnpause = true;
					move_cmd.linear.x = 0.0;
					move_cmd.angular.z = 0.0;
					cmd_vel_pub.publish(move_cmd);
					clearLights = true;
					setLights(backToNormalLights);
				}
				else
				{
					listeningForUnpause = false;
					unpaused = false;
					clearLights = false;
					returnTrip = true;
					doorReached = false;
					currentGoalX = startDoorX;
					currentGoalY = startDoorY;
					currentGoalOrientation = tf::createQuaternionMsgFromYaw(startDoorYaw);
					std_msgs::Header tmpHead;
					geometry_msgs::Pose tmpPose;
					geometry_msgs::Point tmpPoint;
					geometry_msgs::Quaternion tmpQuaternion;
					tmpPoint.x = startDoorX;
					tmpPoint.y = startDoorY;
					tmpPoint.z = 0.0;
					tmpQuaternion = currentGoalOrientation;
					tmpPose.position = tmpPoint;
					tmpPose.orientation = tmpQuaternion;
					goal_cmd.pose =	tmpPose;
					tmpHead.frame_id = mapFrame;
					goal_cmd.header = tmpHead;
					if (oneRun == false)
					{
						goal_pub.publish(goal_cmd);
					}
				}

				
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
			if ((fabs(currentX - startDoorX) < 0.4 && fabs(currentY - startDoorY) < 0.4) &&  (fabs(yawDiff) < 0.3))
			{
			
				doorReached = true;
				currentGoalX = startX;
				currentGoalY = startY;
				currentGoalOrientation = tf::createQuaternionMsgFromYaw(startYaw);

				std_msgs::Header tmpHead;
				geometry_msgs::Pose tmpPose;
				geometry_msgs::Point tmpPoint;
				geometry_msgs::Quaternion tmpQuaternion;
				tmpPoint.x = startX;
				tmpPoint.y = startY;
				tmpPoint.z = 0.0;
				tmpPose.position = tmpPoint;
				tmpQuaternion = currentGoalOrientation;
				tmpPose.orientation = tmpQuaternion;
				goal_cmd.pose =	tmpPose;
				tmpHead.frame_id = mapFrame;
				goal_cmd.header = tmpHead;

				goal_pub.publish(goal_cmd);
					
				
			}
		}
		else
		/*On the return trip and have reached the point near the other side of the door, now looking for when you arrive at the start to re-target the goal door*/
		{
			//ROS_INFO("[ASSERTIVE_BEHAVIOUR] Am I in the right place?");
			float yawDiff = tf::getYaw(currentOrientation) - startYaw;
			if (yawDiff > 3.14159)
			{
				yawDiff += -6.2831;
		    	}
			else if (yawDiff < -3.14159)
			{
				yawDiff += 6.2831;
			}
			if (((fabs(currentX - startX) < 0.4 && fabs(currentY - startY) < 0.4) &&  (fabs(yawDiff) < 0.3)) || firstGoal == false)
			{
				if(unpaused == false)
				{

					listeningForUnpause = true;
					move_cmd.linear.x = 0.0;
					move_cmd.angular.z = 0.0;
					cmd_vel_pub.publish(move_cmd);
					clearLights = true;
					setLights(backToNormalLights);
				}
				else
				{
					ROS_INFO("[ASSERTIVE_BEHAVIOUR] Goal sent!");
					firstGoal = true;
					returnTrip = false;
					doorReached = false;
					listeningForUnpause = false;
					unpaused = false;
					clearLights = false;
					currentGoalX = goalDoorX;
					currentGoalY = goalDoorY;
					currentGoalOrientation = tf::createQuaternionMsgFromYaw(goalDoorYaw);

					std_msgs::Header tmpHead;
					geometry_msgs::Pose tmpPose;
					geometry_msgs::Point tmpPoint;
					geometry_msgs::Quaternion tmpQuaternion;
					tmpPoint.x = goalDoorX;
					tmpPoint.y = goalDoorY;
					tmpPoint.z = 0.0;
					tmpPose.position = tmpPoint;
					tmpQuaternion = currentGoalOrientation;
					tmpPose.orientation = tmpQuaternion;
					goal_cmd.pose =	tmpPose;
					tmpHead.frame_id = mapFrame;
					goal_cmd.header = tmpHead;

					goal_pub.publish(goal_cmd);
				}

				
			}
		}
	}		
}



void Assert2Behaviour::spinOnce() {
	ros::Rate rate(loopHz);
	
	if (stageMode == true)
	{
		unpaused = true;
	}


	if (laserReceived == true && poseReceived == true && subjectPoseReceived == true)
	{
		
		if((timer + ros::Duration(5) < ros::Time::now()) && firstTime == false)
		{
			audio_cmd.data = startupSound;
			audio_pub.publish(audio_cmd);
			setLights(startupLights);
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
	while (ros::ok()) {
		spinOnce();
  	}
}
