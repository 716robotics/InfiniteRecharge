//FRC team 716 Runabout value tuning file
//Created: Jan 10, 2020

//Motor Speeds
#define DRIVESPEED  1
#define POSCTLSPD   0.35
#define ROTCTLSPD   0.45
#define CLIMBEXT    0.6
#define CLIMBRET    -0.8
#define SHOOTSPD    -1
#define AUTOSHOOTSPEED -1.0
#define PICKUPSPEED 0.5
#define BELTz1SPD   -0.8
#define BELTz1SPDs   -0.8
#define BELTz2SPDp   0
#define BELTz2SPDs   -0.8
#define BELTz3SPD   -1
#define BELTz3SPDs   -1

//Timer Values
#define POSTIMEOUT  18
#define ROTTIMEOUT  20
#define SHOOTSPINUP 2.5 //determines Shooter Delay in AUTO
// Autonomous Timer Values
#define AUTOSHOOTTIME 1.5

//Control System Values
#define GPTRIGGERDEAD 0.2 //GamePad Trigger Deadzone

//Servo Defines
#define SRVOUNLOCKED 0
#define SRVOLOCKED   180

//Encoder Targets
#define HIGHGOALTARGET      2000
#define LOWGOALTARGET       1000
#define SHOOTPOSTOLERANCE   1

#define FORWARD 1
#define AUTOFORWARD 1

// DistanceDrive values
#define AUTOSTARTSPEED 0.4
#define DRIVERAMPUPDISTANCE 20
//? get values from previous year's code


// OrientRobot return values
#define NOTDONEYET 0
#define DONE 1
// DistanceDrive return value (also uses OrientRobot return values)
#define ERROR -1

// Autonomous Start Values
#define STARTTRENCH 0
#define STARTGOAL 2
#define STARTCENTER 3

// Autonomous Initial Shooting
#define AUTOSHOOTGoalAngle      0
#define AUTOSHOOTTrenchAngle  -30
#define AUTOSHOOTMidfieldAngle 29
// Shooting distances
#define AUTOSHOOTGoalDistance  120
#define AUTOSHOOTTrenchDistance 138
#define AUTOSHOOTMidfieldDistance 137

// Auonomous Move Across Field
#define AUTOCROSSMidfieldAngleStep0 180
#define AUTOCROSSGoalAngleStep0 170
#define AUTOCROSSTrenchAngleStep0 -147
#define AUTOCROSSMidfieldDistStep1 350
#define AUTOCROSSGoalDistStep1 160
#define AUTOCROSSTrenchDistStep1 189
#define AUTOCROSSGoalAngleStep2 -160
#define AUTOCROSSTrenchAngleStep2 157.5
#define AUTOCROSSGoalDistStep2 200
#define AUTOCROSSTrenchDistStep2 208

// Auonomous Pickup Along Trench
// 162" to center field - 28" to center trench
#define AUTOTRENCHMidfieldToTrenchDist 134
// 96" to center goal - 28" to center trench
#define AUTOTRENCHGoalToTrenchDistance 68
//#define AUTOTRENCHDistAlongTrench 176
#define AUTOTRENCHDistAlongTrench 136
#define AUTOTRENCHDistAlongTrench2 176

#define AUTOTRENCHShootAngle -12
#define AUTOPICKUPSTARTTIME 0

// Autonomous Pickup from midfield 
// looking to line up with opponent load station 
// then move perpendicular into midfield
// opponent load station centered 221" from our trench side wall
#define AUTOMIDDLEMidfieldToTurnPoint 59
#define AUTOMIDDLEGoalToTurnPoint 125
#define AUTOMIDDLETrenchToTurnPoint 193
#define AUTOMIDDLEDistToMiddle 140
#define AUTOMIDDLEDistCollect 84

// 9000 count for 120 in, multiplied by 4 (quad mode)
#define ROBOTDISTANCEPERPULSE 0.05333333
// value to add to curvature correction to drive straight
#define AUTOCURVECOMP -0.066667
