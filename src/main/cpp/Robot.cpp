// 716 robot: Runabout
// Created January 2020

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <Mappings.h>
#include <Tuning.h>

void Robot::RobotInit() {
  au_StartPos.SetDefaultOption(st_Goal, st_Goal);
  au_StartPos.AddOption(st_Trench_A, st_Trench_A);
  au_StartPos.AddOption(st_Midfield, st_Midfield);
  au_StartPos.AddOption(st_Test, st_Test);
  au_MidStart.SetDefaultOption(trench_cells, trench_cells);
//  au_MidStart.AddOption(middle_cells, middle_cells);
  au_MidStart.AddOption(off_line, off_line);
//  au_MidStart.AddOption(vs_field, vs_field);
  //au_StartPos.AddDefault(test_orientation, test_orientation);
  frc::SmartDashboard::PutData("Auto Modes", &au_StartPos);
  frc::SmartDashboard::PutNumber("Auto Delay", 0);
  frc::SmartDashboard::PutData("Auto Next", &au_MidStart);
  colorSensorArm.Set(colorSensorArm.kReverse); //initalize the actuator
  bottomTension.Set(bottomTension.kForward);
  topTension.Set(topTension.kReverse);
  ahrs = new AHRS(SPI::Port::kMXP);
  compressor.Start();
  ahrs->Reset();
}

void Robot::RobotPeriodic() 
{
  frc::SmartDashboard::PutNumber("Left Drive", leftDriveEncoder.GetRaw());
  frc::SmartDashboard::PutNumber("Right Drive", rightDriveEncoder.GetRaw());
  frc::SmartDashboard::PutNumber("Shoot Pos", ShootPosEncoder.GetRaw());
  frc::SmartDashboard::PutNumber("State", rs);
}

void Robot::AutonomousInit() {
  au_SelectedStartPos = au_StartPos.GetSelected();
  std::cout << "Auto selected: " << au_SelectedStartPos << std::endl;
  frc::SmartDashboard::PutString("Selected Auto", au_SelectedStartPos);
  frc::SmartDashboard::GetNumber("Auto Delay", au_AuToDeLay);
  au_SelectedMidStart = au_MidStart.GetSelected();
  std::cout << "Next Action"  << au_SelectedMidStart << std::endl;
  frc::SmartDashboard::PutString("Next Action", au_SelectedMidStart);
  ahrs->Reset();
  leftDriveEncoder.Reset();
	rightDriveEncoder.Reset();
	leftDriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
	rightDriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
  compressor.Start();
  AutoTimer.Start();
}

void Robot::AutonomousPeriodic() {
static bool FirstCallFlag = true;  
static AutoStateType as = as_initialState;
int rc;
static double Angle; 
static double joe;      // Initial timer value for pause
static int whichStep;   // Used within each state to track interim step
static int StartPos;
static int collectorRC; 
static int orientRC;
static int spinupRC;
static int shootRC;
static int driveRC;
static int hoodRC;
static int testRC1 = NOTDONEYET;
static int testRC2 = NOTDONEYET;
static int testRC3 = NOTDONEYET;
static double shootDistance;
float distance;
if ((as == as_initialState) && (au_SelectedStartPos == st_Test)) {
   as = as_testState;
} 

SmartDashboard::PutNumber("AutoPeriodic current angle ", ahrs->GetYaw());

switch(as)
{
  case as_initialState:
    // Do initial autonomous stuff - could be moved to AutonomousInit
    if (au_SelectedStartPos == st_Goal){
       Angle = AUTOSHOOTGoalAngle;
       StartPos = STARTGOAL;
       shootDistance = AUTOSHOOTGoalDistance;        
    }
    else if (au_SelectedStartPos == st_Trench_A){
      Angle = AUTOSHOOTTrenchAngle;
      shootDistance = AUTOSHOOTTrenchDistance;        
      StartPos = STARTTRENCH;
    }
    else if (au_SelectedStartPos == st_Midfield){
      Angle = AUTOSHOOTMidfieldAngle;
      shootDistance = AUTOSHOOTMidfieldDistance;        
      StartPos = STARTCENTER;
    }  
    else 
      Angle = AUTOSHOOTGoalAngle;
    as = as_orientState;
    break;

  case as_orientState:
    // Orient Robot for shooting based on start position entered on smart dashboard
    if (FirstCallFlag == true) {
      FirstCallFlag = false;
      // Don't bother trying to orient to 0 - we're already there
      if (Angle != 0) {
        orientRC = NOTDONEYET;
      } else {
        orientRC = DONE;
      }
      spinupRC = NOTDONEYET;
    } // end (FirstCallFlag)
    if (orientRC == NOTDONEYET) {
      orientRC = OrientRobotSlow(Angle);
    }
    if (spinupRC == NOTDONEYET) {
//      spinupRC = AutonomousSpinUp(shootDistance);
      spinupRC = AutonomousSpinUp(0);
    }
    if ((orientRC == DONE) && (spinupRC == DONE)){
      as = as_shootState;
      FirstCallFlag = true;
    }
    break; // end of case OrientState
    
  case as_shootState:
    // Shoot power cells
    shootRC = AutonomousShoot();
    if (shootRC == DONE) {
      as = as_pauseState;
      FirstCallFlag = true;
    }
    break; // end case shootState

  case as_pauseState:
    // Optional pause entered on smart dashboard - can be 0
    if (FirstCallFlag == true) {
      joe = AutoTimer.Get();
      FirstCallFlag = false;
    }
    // if pause duration is 0, we'll immediately hit this condition
    if ((AutoTimer.Get() - joe) >= au_AuToDeLay) {
      // Setup next state, depending on smart dashboard input
      FirstCallFlag = true;
      if (au_SelectedMidStart == trench_cells)
        as = as_trenchState;
//      else if (au_SelectedMidStart == middle_cells)
//        as = as_middleState;
      else if (au_SelectedMidStart == off_line)
        as = as_moveofflineState;
//      else if (au_SelectedMidStart == vs_field)
//        as = as_enemyState;
      else 
        as = as_moveofflineState;
    } 
    break;

  case as_trenchState:
    // Pick up power cells along alliance trench
    if (FirstCallFlag == true){
      FirstCallFlag = false;
      whichStep = 0;
    }
    switch (whichStep){
      case 0:
        // Turn toward trench
        if (StartPos == STARTTRENCH)
          whichStep = 2;
        else { 
          orientRC = OrientRobot(90);
          if (orientRC == NOTDONEYET)
            orientRC = OrientRobot (90);
          if (orientRC == DONE)
            whichStep = 1;
          } 
        break; // end trench state step 0
      case 1:
        // After rotating, drive along start line to trench
        if (StartPos == STARTCENTER)
          distance = AUTOTRENCHMidfieldToTrenchDist;
        else
          distance = AUTOTRENCHGoalToTrenchDistance;
        rc = DistanceDrive(0.6, distance, 90, true);
        if (rc == DONE)
          whichStep = 2;
        else if (rc == ERROR)
          as = as_finalState; // Stop on an encoder error
        break; // end trench state step 1
      case 2:
        // Once in line with the trench, rotate towards the powercells to collect
        rc = OrientRobot(180);
        if (rc == DONE) {
          whichStep = 3;
          driveRC = NOTDONEYET;
          collectorRC = NOTDONEYET;
        }  
        break;  // end trench state step 2
      case 3:
        // Start collector 
        rc = StartCollector();
        if (rc == DONE) {
          hoodRC = NOTDONEYET;
          whichStep = 4;
        }
        break;  // end trench state step 3

      case 4:
        // Move along trench toward power cells and raise hood
        rc = DistanceDrive(0.55, AUTOTRENCHDistAlongTrench2, 180, true);
        if (rc == DONE)
          whichStep = 5;
        else if (driveRC == ERROR)
           as = as_finalState;
        break;

      case 5:
        // Stop pickup motors
        StopCollector();
        
        orientRC = NOTDONEYET;
        spinupRC = NOTDONEYET;
        hoodRC = NOTDONEYET;
        whichStep = 106;
        break;

      case 6:
        // Aim at goal and start up motors and adjust hood 
        if (orientRC == NOTDONEYET) {
          orientRC = OrientRobotSlow(AUTOTRENCHShootAngle);
          if (orientRC == DONE) {
            joe = AutoTimer.Get();
          }
        }
        if (spinupRC == NOTDONEYET) {
          // spinupRC = AutonomousSpinUp(shootDistance);
          spinupRC = AutonomousSpinUp(1);
        }
        if (hoodRC == NOTDONEYET) {
          hoodRC = DONE;
        }
        // Wait for motor to spin up, robot to be in position, hood to be moved AND
        // at least 1 second for motor spin up after rotation for max shooter speed
        if ((orientRC == DONE) && 
            (spinupRC == DONE) && 
            (hoodRC == DONE) && 
            (((AutoTimer.Get() - joe) >= 1.0))){
     
          whichStep = 7;
        }
        break;
      
      case 106:
        // Aim at goal and start up motors and adjust hood 
        if (orientRC == NOTDONEYET) {
          orientRC = OrientRobotSlow(AUTOTRENCHShootAngle);
        }
        if (spinupRC == NOTDONEYET) {
          // spinupRC = AutonomousSpinUp(shootDistance);
//? spinup raises the hood
          spinupRC = AutonomousSpinUp(0); //changes hood angle
        }
		if (orientRC == DONE) {
			if (driveRC == NOTDONEYET) {
				driveRC = DistanceDrive(0.55, 60, AUTOTRENCHShootAngle, false);
			} 
		}
        // By definition, driveRC cannot be DONE until orientRC is DONE, 
		// so no need to check that
		if ((spinupRC == DONE) && (driveRC == DONE)){
          whichStep = 7;
        }
        break;
      
            case 7:
        // Shoot power cells
        rc = AutonomousShoot();
        if (rc == DONE) {
          whichStep = 8;
        }
      break;

      case 8:
        // Once in line with the trench, rotate towards the powercells to collect
        rc = OrientRobot(180);
        if (rc == DONE) {
          as = as_finalState;
        }  
        break;  // end trench state step 2
    }
    break; // end of case trenchState 


  case as_moveofflineState:
    // Get off the starting line
    if (FirstCallFlag == true){
      FirstCallFlag = false;
      whichStep = 0;
    }
    switch (whichStep){
      case 0:
        // Turn around and face alliance wall. We could move offline without rotating,
        // but want to be facing our loading station and/or middle of field for power
        // cell pickup
        rc = OrientRobot(180);
        if (rc == DONE)
          whichStep = 1;
        break; // end move offline state step 0
      case 1:
        rc = DistanceDrive(0.7, 36, 180, true);
        if ((rc == DONE) || (rc == ERROR)) {
          // If done moving off line OR an error occurs, we're done with autonomous
          FirstCallFlag = true;
          as = as_finalState;
        }
        break; // end move offline state step 1
    }
    break; // end move offline state
  
  case as_testState:
    // Change as needed
    //rc = DistanceDrive(0.6, 120, 0, true);
    if (testRC1 == NOTDONEYET)
       testRC1 = OrientRobot(179);
    else if (testRC2 == NOTDONEYET) {
      testRC2 = OrientRobotSlow(-16);
      spinupRC = AutonomousSpinUp(1.4);
    }

    if ((testRC2 == DONE) && (spinupRC == DONE))
//    if (testRC1 == DONE)
       as = as_finalState;
    break;

  case as_finalState:
  default:
    // Finish up autonomous
    // Shut off all motors
    AutoTimer.Stop();
    cpMotor.Set(0);
    climb.Set(0);
    pickupM.Set(0);
    ShootPosMotor.Set(ShootPosMotor.kOff);
    ShootWhl.Set(0);
    BeltZ1.Set(0);
    BeltZ2.Set(0);
    BeltZ3.Set(0);
    colorSensorArm.Set(colorSensorArm.kReverse);
    pickupPneumatic.Set(pickupPneumatic.kReverse);
    bottomTension.Set(bottomTension.kForward);
    topTension.Set(topTension.kReverse);
    break; // end of final state
  }
}

void Robot::TeleopInit() 
{
  frc::SmartDashboard::PutString("Position Control Status", "Not Ready");
  PosTimeout.Reset();
  RotTimeout.Reset();
  ShootWhl.Set(0);
  ShootPosMotor.Set(ShootPosMotor.kOff);
  BeltZ1.Set(0);
  BeltZ2.Set(0);
  BeltZ3.Set(0);
  bottomTension.Set(bottomTension.kForward);
  topTension.Set(topTension.kReverse);
  rs = rs_main;
  leftDriveEncoder.Reset();
	rightDriveEncoder.Reset();
	leftDriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
	rightDriveEncoder.SetDistancePerPulse(ROBOTDISTANCEPERPULSE);
  climbSRVO.SetAngle(SRVOUNLOCKED);
  ahrs->Reset();
}

void Robot::TeleopPeriodic() 
{
frc::SmartDashboard::PutNumber("Match Time", frc::Timer::GetMatchTime());
if (leftDriveStick.GetTrigger()) StraightDrive();
else {drive.TankDrive((leftDriveStick.GetY() * -1), (rightDriveStick.GetY() * -1));
sdfr = false;}
if (gamepad.GetBackButtonPressed()) Abort();
frc::SmartDashboard::PutNumber("State", rs);

SmartDashboard::PutNumber(  "Current Angle", ahrs->GetYaw());

switch(rs)
{
  case rs_main:
    TeleopMain();
    break;
  case rs_AutoPos:
    PositionControl();
    break;
  case rs_AutoRot:
    RotationControl();
    break;
  case rs_manualCPL:
    ManualRotation();
    break;
  case rs_pickup:
    PickUp();
    break;
  case rs_Shoot:
    Shoot();
    break;
  case rs_climb:
    Climbing();
    break;
    default:
      frc::DriverStation::ReportError("Robot has entered an unknown state!");
      break;
}
if (doConfirmRumble) confirmRumble();
if (doFatalRumble) FatalErrorRumble();
if (doClimbErrRumble) ClimbErrorRumble();
if (doShootErrRumble) ShooterErrorRumble();
if (doPickupErrRumble) PickupErrorRumble();
if (doCPerrRumble) CPerrorRumble();
if (gamepad.GetRawButton(8) && !doLowPower) doLowPower = true;
  else if (gamepad.GetRawButton(8) && doLowPower) {doLowPower = false;
  compressor.Start();}
if(doLowPower) LowPower();
if(doPickupRetract) RetractPickUp();
}
void Robot::TeleopMain()
{
if (gamepad.GetYButton() || rightDriveStick.GetRawButton(5)) rs = rs_AutoPos;
if (gamepad.GetXButton()|| rightDriveStick.GetRawButton(4)) rs = rs_AutoRot;
if (gamepad.GetTriggerAxis(gamepad.kRightHand) > 0.8) rs = rs_pickup;
if (gamepad.GetBumper(gamepad.kRightHand)) doPickupRetract = true;
if (gamepad.GetAButtonPressed() &&((frc::Timer::GetMatchTime() <= 35) || allowclimboverride)) rs = rs_climb;
if (gamepad.GetStickButton(gamepad.kRightHand) && rs != rs_manualCPL) rs = rs_manualCPL;
if (gamepad.GetBumper(gamepad.kLeftHand) && allowshChange) {rs = rs_Shoot; //if left or right on hat
  allowshChange = false;}
else if (gamepad.GetBumper(gamepad.kLeftHand) == 0) allowshChange = true;
colorSensorArm.Set(colorSensorArm.kReverse);
if (gamepad.GetAButton() && gamepad.GetBButton()){
  static int overridecount =0;
  overridecount ++;
  if (overridecount == 100) {
    allowclimboverride = true;
    doConfirmRumble = true;}
}
}
void Robot::TestPeriodic() {
drive.TankDrive((leftDriveStick.GetY() * -1), (rightDriveStick.GetY() * -1));
if (gamepad.GetBackButtonPressed()) Abort();
if (gamepad.GetPOV() != -1){
  if (gamepad.GetPOV() == 180) BeltZ1.Set(-1);
  if (gamepad.GetPOV() == 270) BeltZ2.Set(1);
  if (gamepad.GetPOV() == 0) BeltZ3.Set(1);
}
else{
  BeltZ1.Set(0);
  BeltZ2.Set(0);
  BeltZ3.Set(0);
  }
  if (gamepad.GetAButton()){
      pickupPneumatic.Set(pickupPneumatic.kForward);
      pickupM.Set(1);
  }
  else{
    pickupPneumatic.Set(pickupPneumatic.kReverse);
      pickupM.Set(0);
  }
  if (gamepad.GetTriggerAxis(gamepad.kRightHand) > 0.5) ShootWhl.Set(-1);
  else ShootWhl.Set(0);
  if(gamepad.GetY(gamepad.kLeftHand) > 0.6) ShootPosMotor.Set(ShootPosMotor.kReverse);
  else if(gamepad.GetY(gamepad.kLeftHand) < -0.6) ShootPosMotor.Set(ShootPosMotor.kForward);
  else ShootPosMotor.Set(ShootPosMotor.kOff);
  if(gamepad.GetXButton()){
    topTension.Set(topTension.kForward);
    bottomTension.Set(bottomTension.kForward);
  }
  else {
    topTension.Set(topTension.kReverse);
    bottomTension.Set(bottomTension.kReverse);
  }
}

void Robot::StraightDrive(){
//drive.TankDrive
if (!sdfr){
  leftDriveEncoder.Reset();
  rightDriveEncoder.Reset();
  sdfr = true;
}
double throttle = (-1 *leftDriveStick.GetY());
double left = (leftDriveEncoder.GetDistance());
double right = (-1 * rightDriveEncoder.GetDistance());
double difference = right - left;
double lpower = (throttle - (difference * 0.1));
double rpower = (throttle + (difference * 0.1));
drive.TankDrive(lpower, rpower, false);
}

/* This Function communicates with the I2C color sensor to get RGB values
* 0 = no color detected, 1 = red, 2 = green, 3 = cyan, 4 = yellow
*/
int Robot::IdentifyColor()
{
  colorSensorArm.Set(colorSensorArm.kForward);
  frc::Color detectedColor = colorSensor0.GetColor();
  int C_color = 0;
  double C_red = detectedColor.red * 255;
  double C_green = detectedColor.green * 255;
  double C_blue = detectedColor.blue * 255;
  if (((C_red > C_green) &&(C_green > C_blue)) || ((C_green > C_red) && (C_red > C_blue))) C_color = 1; //red
  if ((C_green > C_blue) && (C_blue > C_red))
  {
    if ((C_green - C_blue) > 40) C_color = 2; //green
    else C_color = 3; //blue
  }
  if ((C_blue > C_green) && (C_green > C_red)) C_color = 3; //also blue
  if (((C_green > C_red) &&(C_red > C_blue))&& (C_green >= 138))C_color = 4; // yellow
  double proximity = colorSensor0.GetProximity(); // The closest reading is 2047 and furthest is 0
  frc::SmartDashboard::PutNumber("Proximity", proximity);
  frc::SmartDashboard::PutNumber("Red", (detectedColor.red * 255));
  frc::SmartDashboard::PutNumber("Green", (detectedColor.green * 255));
  frc::SmartDashboard::PutNumber("Blue", (detectedColor.blue * 255));
  if (proximity > 400) frc::SmartDashboard::PutString("Color Status", "OK");
  else {
    frc::SmartDashboard::PutString("Color Status", "Not Close Enough"); 
    C_color = 0;
  }
  frc::SmartDashboard::PutNumber("Color Detected", C_color);
  return C_color;
}
void Robot::PositionControl()
{
  std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  if (PosTimeout.Get() == 0) PosTimeout.Start();
  static int target = 0;
  static float speed = 1;
  static bool a = false;
  if (target == 0) {
    switch (gameData[0])
    {
      case 'B':
        target = 1;
        break;
      case 'G':
        target = 4;
        break;
      case 'R':
        target = 3;
        if (!a) {speed = speed - 0.2;
        a = true;}
        break;
      case 'Y':
        target = 2;
        break;
      default:
        frc::DriverStation::ReportError("Position Control Attempted but No Game Data was Recieved!");
        rs = rs_main;
	doCPerrRumble = true;
        break;
    }}
    frc::SmartDashboard::PutNumber("Target", target);
    if (IdentifyColor() != target) cpMotor.Set(POSCTLSPD * speed);
    else if (IdentifyColor() == target){
       cpMotor.Set(-0.2 * speed);
      frc::SmartDashboard::PutString("Position Control Status", "Complete");
      PosTimeout.Stop();
      PosTimeout.Reset();
      frc::Wait(0.2);
      cpMotor.Set(0);
      frc::Wait(0.5);
      if (IdentifyColor() == target) {rs = rs_main;
      speed = 1;}
      else speed = (speed * -0.7);
    }
    if (Robot::PosTimeout.Get() > POSTIMEOUT)
    {cpMotor.Set(0);
    speed = 1;
    frc::SmartDashboard::PutString("Position Control Status", "Timed Out. Manual Control Requried");
    PosTimeout.Stop();
    PosTimeout.Reset();
    rs = rs_main;
    a = false;}
}
void Robot::RotationControl(){
  static int LastColor = 0;
  static int rotCount = 0;
  frc::SmartDashboard::PutNumber("rotCount", rotCount);
  frc::SmartDashboard::PutNumber("LastColor", LastColor);
  if (RotTimeout.Get() == 0) RotTimeout.Start();
  frc::SmartDashboard::PutNumber("LastColor", LastColor);
  if ((IdentifyColor() != LastColor)){
    LastColor = IdentifyColor();
    if (IdentifyColor() == 1) rotCount ++;
  }
 if ((rotCount >= 8) || (RotTimeout.Get() >= ROTTIMEOUT)) {
    cpMotor.Set(0);
    rs = rs_main;
    frc::SmartDashboard::PutString("Auto Rotate Status", "Done");
    RotTimeout.Stop();
    RotTimeout.Reset();
    rotCount = 0;
  } else {
    cpMotor.Set(ROTCTLSPD);
    frc::SmartDashboard::PutString("Auto Rotate Status", "Working...");
  }
}
void Robot::PickUp(){
  static bool a = false; //keeps us from constantly setting motors to the same thing
  static bool ballCurrentlyIn = false;
  static int LastForeSensorState = 0;
  static int LastAftSensorState = 0;
  static int fBallsLast = fBalls;
  if (!a) {
    pickupM.Set(PICKUPSPEED);
    pickupPneumatic.Set(pickupPneumatic.kForward);
    BeltZ1.Set(BELTz1SPD);
    BeltZ2.Set(BELTz2SPDp);
    BeltZ3.Set(0);
    topTension.Set(topTension.kReverse);
    bottomTension.Set(bottomTension.kForward);
    LastForeSensorState = ForeBallSensor.Get();
    LastAftSensorState = AftBallSensor.Get();
    fBallsLast = fBalls;
    a = true;
  }
  if (gamepad.GetTriggerAxis(gamepad.kRightHand) < 0.8) {
    a = false;
    rs = rs_main;
    BeltZ1.Set(0);
    BeltZ2.Set(0);
    BeltZ3.Set(0);
    bottomTension.Set(bottomTension.kForward);
  }
}
void Robot::RetractPickUp(){
  static frc::Timer retractTimer;
  if (retractTimer.Get() == 0){
  pickupPneumatic.Set(pickupPneumatic.kReverse);
  pickupM.Set(-1);
  retractTimer.Start();}
  else if (retractTimer.Get() > 2) {pickupM.Set(0);
  retractTimer.Stop();
  retractTimer.Reset();
  doPickupRetract = false;}
}
void Robot::Shoot(){
  static bool done = false;
  static bool didRumble = false;
  static bool allowManualOrient = false;
  static int LastAftSensorState = AftBallSensor.Get();
  static int loops = 0;
  static double shootSPD = SHOOTSPD;
  static frc::Timer shootTimer;
  double difference = 0;
  if (shootTimer.Get() > 4 && !didRumble){
    doShootErrRumble = true;
    didRumble = true;
  }
  if (gamepad.GetBumper(gamepad.kLeftHand) == 0) allowshChange = true;
  if (gamepad.GetBumper(gamepad.kLeftHand) == 1 && allowshChange) done = true;
  ShootWhl.Set(shootSPD);
  if (loops == 0) {
    shootTimer.Start();
    shootSPD = SHOOTSPD;
  frc::SmartDashboard::PutString("Shoot Status", "Spinning Up...");
  allowManualOrient = true;
    BeltZ1.Set(0);
    BeltZ2.Set(0);
    BeltZ3.Set(0);
    bottomTension.Set(bottomTension.kReverse);
    topTension.Set(topTension.kForward); //both tensioners engaged
    gamepad.SetRumble(gamepad.kLeftRumble, 0.1);
    done = false;
    doPickupRetract = true;
  }
  if (gamepad.GetTriggerAxis(gamepad.kLeftHand) > 0.8){
    ShootPosMotor.Set(ShootPosMotor.kOff);
    //if (shootSPD > -1) shootSPD = shootSPD - 0.0015; //ramping behavior
    BeltZ1.Set(BELTz1SPDs);
    BeltZ2.Set(BELTz2SPDs);
    BeltZ3.Set(-1 * BELTz3SPDs);
    gamepad.SetRumble(gamepad.kLeftRumble, 0);
  }
  else{
    if (allowManualOrient){
      if(gamepad.GetY(gamepad.kLeftHand) > 0.6) ShootPosMotor.Set(ShootPosMotor.kReverse);
      else if(gamepad.GetY(gamepad.kLeftHand) < -0.6) ShootPosMotor.Set(ShootPosMotor.kForward);
      else ShootPosMotor.Set(ShootPosMotor.kOff);
    }
  } 
  loops ++;
  if (gamepad.GetPOV() != -1){
    BeltZ1.Set(1);
    BeltZ2.Set(1);
    BeltZ3.Set(-1);
    frc::Wait(0.25);
    BeltZ1.Set(0);
    BeltZ2.Set(0);
    BeltZ3.Set(0);
  }
  if (done){
    rs = rs_main;
    ShootPosMotor.Set(ShootPosMotor.kOff);
    ShootWhl.Set(0);
    BeltZ1.Set(0);
    BeltZ2.Set(0);
    BeltZ3.Set(0);
    //bottomTension.Set(bottomTension.kForward);
    topTension.Set(topTension.kReverse);
    fBalls = 0;
    aBalls = 0;
    allowshChange = false;
    loops = 0;
    gamepad.SetRumble(gamepad.kLeftRumble, 0);
    didRumble = false;
    shootTimer.Stop();
    shootTimer.Reset();
  }
 }
void Robot::ManualRotation(){
  IdentifyColor();
  frc::SmartDashboard::PutString("Auto Rotate Status", "MANUAL OVERRIDE");
  frc::SmartDashboard::PutString("Position Control Status", "MANUAL OVERRIDE");
  cpMotor.Set(gamepad.GetX(gamepad.kRightHand));
}
void Robot::Climbing(){
  static bool lockout = false;
  if (leftDriveStick.GetRawButtonPressed(6) && rightDriveStick.GetRawButtonPressed(11)) lockout = true;
  if (!lockout){
  if (gamepad.GetAButton()) climb.Set(CLIMBEXT);
  else if (gamepad.GetBButton()) climb.Set(CLIMBRET);
    else climb.Set(0);
  }
  else {
    climb.Set(0);
    climbSRVO.Set(SRVOLOCKED);
    if (gamepad.GetAButton()) doClimbErrRumble = true;
    else if (gamepad.GetBButton()) climb.Set(CLIMBRET);
    else climb.Set(0);
  }
}
void Robot::Abort(){
rs = rs_main;
frc::SmartDashboard::PutString("Auto Rotate Status", "ABORTED");
frc::SmartDashboard::PutString("Position Control Status", "ABORTED");
cpMotor.Set(0);
climb.Set(0);
pickupM.Set(0);
ShootPosMotor.Set(ShootPosMotor.kOff);
ShootWhl.Set(0);
BeltZ1.Set(0);
BeltZ2.Set(0);
BeltZ3.Set(0);
gamepad.SetRumble(gamepad.kRightRumble, 0);
gamepad.SetRumble(gamepad.kLeftRumble, 0); 
colorSensorArm.Set(colorSensorArm.kReverse);
pickupPneumatic.Set(pickupPneumatic.kReverse);
bottomTension.Set(bottomTension.kForward);
topTension.Set(topTension.kReverse);
fBalls = 0;
aBalls = 0;
PosTimeout.Stop();
PosTimeout.Reset();
RotTimeout.Stop();
RotTimeout.Reset();
RumbleTimer.Stop();
RumbleTimer.Reset();
doLowPower = false;
doPickupRetract = false;
doConfirmRumble = false;
doFatalRumble = false;
doClimbErrRumble = false;
doShootErrRumble = false;
doPickupErrRumble = false;
doCPerrRumble = false;
}
void Robot::LowPower(){
  compressor.Stop();
  cpMotor.Set(0);
  //turn off roller system
}
void Robot::CPerrorRumble()
{ 
  if (RumbleTimer.Get() == 0){
    RumbleTimer.Start();
    gamepad.SetRumble(gamepad.kRightRumble, 0);
    gamepad.SetRumble(gamepad.kLeftRumble, 0.5);}
  if ((RumbleTimer.Get() >= 0.5) && (RumbleTimer.Get() < 1)){
    gamepad.SetRumble(gamepad.kRightRumble, 0.5);
    gamepad.SetRumble(gamepad.kLeftRumble, 0);}
  if ((RumbleTimer.Get() >= 1) && (RumbleTimer.Get() < 1.5)){
    gamepad.SetRumble(gamepad.kRightRumble, 0);
    gamepad.SetRumble(gamepad.kLeftRumble, 0.5);}
  if ((RumbleTimer.Get() >= 1.5) && (RumbleTimer.Get() < 2)){
    gamepad.SetRumble(gamepad.kRightRumble, 0.5);
    gamepad.SetRumble(gamepad.kLeftRumble, 0);}
  if (RumbleTimer.Get() > 2){
    gamepad.SetRumble(gamepad.kRightRumble, 0);
    gamepad.SetRumble(gamepad.kLeftRumble, 0);
    RumbleTimer.Stop();
    RumbleTimer.Reset();
    doCPerrRumble = false;}
  }
void Robot::FatalErrorRumble(){
  if (RumbleTimer.Get() == 0){
    RumbleTimer.Start();
    gamepad.SetRumble(gamepad.kRightRumble, 1);
    gamepad.SetRumble(gamepad.kLeftRumble, 1);}
  if ((RumbleTimer.Get() >= 3) && RumbleTimer.Get() < 4){
    gamepad.SetRumble(gamepad.kRightRumble, 0.5);
    gamepad.SetRumble(gamepad.kLeftRumble, 0.5);}
  else if (RumbleTimer.Get() >4) RumbleTimer.Reset();
}  
void Robot::ClimbErrorRumble(){
  if (RumbleTimer.Get() == 0){
    RumbleTimer.Start();
    gamepad.SetRumble(gamepad.kRightRumble, 1);
    gamepad.SetRumble(gamepad.kLeftRumble, 0);}
  if ((RumbleTimer.Get() >= 0.5) && RumbleTimer.Get() < 1){
    gamepad.SetRumble(gamepad.kRightRumble, 0);
    gamepad.SetRumble(gamepad.kLeftRumble, 1);}
  else if (RumbleTimer.Get() >= 1) RumbleTimer.Reset();
  }
void Robot::ShooterErrorRumble(){
  if (RumbleTimer.Get() == 0){
    RumbleTimer.Start();
    gamepad.SetRumble(gamepad.kRightRumble, 0.75);
    gamepad.SetRumble(gamepad.kLeftRumble, 1);}
  if ((RumbleTimer.Get() >= 0.5) && RumbleTimer.Get() < 1){
    gamepad.SetRumble(gamepad.kRightRumble, 0.75);
    gamepad.SetRumble(gamepad.kLeftRumble, 1);}
  else if (RumbleTimer.Get() >= 1) {RumbleTimer.Stop();
    RumbleTimer.Reset();
    gamepad.SetRumble(gamepad.kRightRumble, 0);
    gamepad.SetRumble(gamepad.kLeftRumble, 0);  
    doShootErrRumble = false;
 }
}
void Robot::PickupErrorRumble(){
  static int PERcount = 0;
 if (RumbleTimer.Get() == 0){
   RumbleTimer.Start();
   gamepad.SetRumble(gamepad.kRightRumble, 1);
   gamepad.SetRumble(gamepad.kLeftRumble, 0.75);}
 if ((RumbleTimer.Get() >= 0.5) && RumbleTimer.Get() < 1){
   gamepad.SetRumble(gamepad.kRightRumble, 1);
   gamepad.SetRumble(gamepad.kLeftRumble, 0.75);}
else if (RumbleTimer.Get() > 1) {RumbleTimer.Reset();
PERcount ++;}
 else if (PERcount == 4) {RumbleTimer.Stop();
   RumbleTimer.Reset();
   PERcount = 0;
   gamepad.SetRumble(gamepad.kRightRumble, 0);
   gamepad.SetRumble(gamepad.kLeftRumble, 0);
   doPickupErrRumble = false;
 }    
} 
void Robot::confirmRumble(){
  if (RumbleTimer.Get() == 0){
    gamepad.SetRumble(gamepad.kRightRumble, 0.85);
    gamepad.SetRumble(gamepad.kLeftRumble, 0.85);
    RumbleTimer.Start();
  }
  else if (RumbleTimer.Get() > 0.4){
    gamepad.SetRumble(gamepad.kRightRumble, 0);
    gamepad.SetRumble(gamepad.kLeftRumble, 0);
    RumbleTimer.Stop();
    RumbleTimer.Reset();
    doConfirmRumble = false;
  }
}
void Robot::NavX_Diag(){
  SmartDashboard::PutBoolean( "IMU_Connected",        ahrs->IsConnected());
        SmartDashboard::PutNumber(  "IMU_Yaw",              ahrs->GetYaw());
        SmartDashboard::PutNumber(  "IMU_Pitch",            ahrs->GetPitch());
        SmartDashboard::PutNumber(  "IMU_Roll",             ahrs->GetRoll());
        SmartDashboard::PutNumber(  "IMU_CompassHeading",   ahrs->GetCompassHeading());
        SmartDashboard::PutNumber(  "IMU_Update_Count",     ahrs->GetUpdateCount());
        SmartDashboard::PutNumber(  "IMU_Byte_Count",       ahrs->GetByteCount());
        SmartDashboard::PutNumber(  "IMU_Timestamp",        ahrs->GetLastSensorTimestamp());

        /* These functions are compatible w/the WPI Gyro Class */
        SmartDashboard::PutNumber(  "IMU_TotalYaw",         ahrs->GetAngle());
        SmartDashboard::PutNumber(  "IMU_YawRateDPS",       ahrs->GetRate());

        SmartDashboard::PutNumber(  "IMU_Accel_X",          ahrs->GetWorldLinearAccelX());
        SmartDashboard::PutNumber(  "IMU_Accel_Y",          ahrs->GetWorldLinearAccelY());
        SmartDashboard::PutBoolean( "IMU_IsMoving",         ahrs->IsMoving());
        SmartDashboard::PutNumber(  "IMU_Temp_C",           ahrs->GetTempC());
        SmartDashboard::PutBoolean( "IMU_IsCalibrating",    ahrs->IsCalibrating());

        SmartDashboard::PutNumber(  "Velocity_X",           ahrs->GetVelocityX() );
        SmartDashboard::PutNumber(  "Velocity_Y",           ahrs->GetVelocityY() );
        SmartDashboard::PutNumber(  "Displacement_X",       ahrs->GetDisplacementX() );
        SmartDashboard::PutNumber(  "Displacement_Y",       ahrs->GetDisplacementY() );

        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
        /* NOTE:  These values are not normally necessary, but are made available   */
        /* for advanced users.  Before using this data, please consider whether     */
        /* the processed data (see above) will suit your needs.                     */

        SmartDashboard::PutNumber(  "RawGyro_X",            ahrs->GetRawGyroX());
        SmartDashboard::PutNumber(  "RawGyro_Y",            ahrs->GetRawGyroY());
        SmartDashboard::PutNumber(  "RawGyro_Z",            ahrs->GetRawGyroZ());
        SmartDashboard::PutNumber(  "RawAccel_X",           ahrs->GetRawAccelX());
        SmartDashboard::PutNumber(  "RawAccel_Y",           ahrs->GetRawAccelY());
        SmartDashboard::PutNumber(  "RawAccel_Z",           ahrs->GetRawAccelZ());
        SmartDashboard::PutNumber(  "RawMag_X",             ahrs->GetRawMagX());
        SmartDashboard::PutNumber(  "RawMag_Y",             ahrs->GetRawMagY());
        SmartDashboard::PutNumber(  "RawMag_Z",             ahrs->GetRawMagZ());
        SmartDashboard::PutNumber(  "IMU_Temp_C",           ahrs->GetTempC());
        /* Omnimount Yaw Axis Information                                           */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
        AHRS::BoardYawAxis yaw_axis = ahrs->GetBoardYawAxis();
        SmartDashboard::PutString(  "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
        SmartDashboard::PutNumber(  "YawAxis",              yaw_axis.board_axis );

        /* Sensor Board Information                                                 */
        SmartDashboard::PutString(  "FirmwareVersion",      ahrs->GetFirmwareVersion());

        /* Quaternion Data                                                          */
        /* Quaternions are fascinating, and are the most compact representation of  */
        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
        /* from the Quaternions.  If interested in motion processing, knowledge of  */
        /* Quaternions is highly recommended.                                       */
        SmartDashboard::PutNumber(  "QuaternionW",          ahrs->GetQuaternionW());
        SmartDashboard::PutNumber(  "QuaternionX",          ahrs->GetQuaternionX());
        SmartDashboard::PutNumber(  "QuaternionY",          ahrs->GetQuaternionY());
        SmartDashboard::PutNumber(  "QuaternionZ",          ahrs->GetQuaternionZ());
}

//------------------------------------------------------------------------
// OrientRobot rotates the robot in place until it is oriented to the specified
// angle relative to the field (its starting position is 0 degrees).
// Specify a value from -180 to +180 where 0 degrees aims straight away from the
// player station, +90 to the right of the player station, -90 to the left of player
// station and -180 or +180 aims back towards the player station.
//-------------------------------------------------------------------------
//------------------------------------------------------------------------
// OrientRobot rotates the robot in place until it is oriented to the specified
// angle relative to the field (its starting position is 0 degrees).
// Specify a value from -180 to +180 where 0 degrees aims straight away from the
// player station, +90 to the right of the player station, -90 to the left of player
// station and -180 or +180 aims back towards the player station.
//-------------------------------------------------------------------------
int Robot::OrientRobot (float angle)
{
  //rotate to desired angle
  static bool FirstCallFlag = true; // FirstCallFlag should always be set to true when returning DONE
  static int iterations = 0;
  static bool brakingFlag = false;
  static double brakeStartTime;
  static double orientStartTime;
  static float direction = 0;
  static float brakeDirection = 0;
  static float firstMagnitude;
  static float leftBrakeSpeed, rightBrakeSpeed;
  static float lastAngle;
  float  magnitude, difference, speed, leftSpeed, rightSpeed;
  bool   changeDirection = false;

#define ORIENTMINANGLE 45
#define ORIENTMAXSPEED 0.75
#define ORIENTMINSPEED 0.5
#define ORIENTBRAKEFACTOR 15
//#define ORIENTBRAKESTOP (firstMagnitude/ORIENTBRAKEFACTOR)
// force us to keep rotating until within 3 degrees for first test
#define ORIENTBRAKESTOP 9

  if (FirstCallFlag) {
    iterations = 0;
    brakingFlag = false;
    direction = 0;
  }

  if (brakingFlag == false) {
    // This is the normal path - called when we're not braking at the end of a turn

    SmartDashboard::PutNumber("OR NRM current angle ", ahrs->GetYaw());
    //-----------------------------------------------------------------------------------
    // calculate direction and magnitude of remaining turn
    // recalculate direction each time in case robot rotates past target before
    // we catch it. We should start braking early enough that this should never happen,
    // but if it does, we need to change direction
    //-----------------------------------------------------------------------------------
    difference = angle - ahrs->GetYaw();
    if (difference > 180)
      difference = difference - 360;
    else if (difference < -180)
      difference = difference + 360;
    if (difference > 0) {
      if (direction != 1) {
        // direction changed - need to recalculate braking speed
        direction = 1;
        brakeDirection = -1;
        changeDirection = true;
      }
      magnitude = difference;
    } else {
      if (direction != 1) {
        // direction changed - need to recalculate braking speed
        direction = -1;
        brakeDirection = 1;
        changeDirection = true;
      }
      magnitude = -1 * difference;
    }
    SmartDashboard::PutNumber("OR NRM difference", difference);
    SmartDashboard::PutNumber("OR NRM magnitude", magnitude);
	
    //-----------------------------------------------------------------------------------
    // once we get close, brake, to avoid overshooting our target angle, then check again
    // limit the number of times we do this in case it gets in a loop where it keeps going
    // too far
    //-----------------------------------------------------------------------------------
    if (magnitude <= ORIENTBRAKESTOP) {
      brakingFlag = true;
      brakeStartTime = AutoTimer.Get();
      lastAngle = ahrs->GetYaw();
      drive.TankDrive(leftBrakeSpeed, rightBrakeSpeed);
      return NOTDONEYET;
    }

    //-----------------------------------------------------------------------------------
    // Calculate speed based on remaining turn magnitude
    //-----------------------------------------------------------------------------------
    if (magnitude > ORIENTMINANGLE) {
      speed = (magnitude - ORIENTMINANGLE) / (180 - ORIENTMINANGLE);
      speed = speed * (ORIENTMAXSPEED-ORIENTMINSPEED);
      speed = speed + ORIENTMINSPEED;
    } else {
      speed = ORIENTMINSPEED;
    }
    leftSpeed = direction * speed * AUTOFORWARD;
    rightSpeed = -1 * leftSpeed;
	
    drive.TankDrive(leftSpeed, rightSpeed);

    //-----------------------------------------------------------------------------------
    // If this is the first call for a new turn, save initial values for braking
    // If robot rotated past target and changed direction, save updated values
    //-----------------------------------------------------------------------------------
    if ((FirstCallFlag == true) || (changeDirection == true)) {
      firstMagnitude = magnitude;
      leftBrakeSpeed = rightSpeed;
      rightBrakeSpeed = leftSpeed;
      FirstCallFlag = false;
      if (FirstCallFlag == true)
        // Don't reset timer on direction change, only first call
        orientStartTime = AutoTimer.Get();
    }

  } else {
    //-----------------------------------------------------------------------------------
    // Braking
    // Compare current angle to last angle to see if robot has reversed direction from
    // direction of rotation
    //-----------------------------------------------------------------------------------
    difference = ahrs->GetYaw() - lastAngle;
    if (difference > 180)
      difference = difference - 360;
    else if (difference < -180)
      difference = difference + 360;
    //-----------------------------------------------------------------------------------
    // Positive difference means we've rotated clockwise since braking
    // Negative difference means we've rotated counterclockwise since braking
    //-----------------------------------------------------------------------------------
    if (((difference >= 0) && (direction == 1)) ||
        ((difference <= 0) && (direction == -1))) {
      //---------------------------------------------------------------------------------
      // We are still rotating in direction of pre-braking rotation
      //---------------------------------------------------------------------------------
      lastAngle = ahrs->GetYaw();
      drive.TankDrive(leftBrakeSpeed, rightBrakeSpeed);
      return NOTDONEYET;
    }
    else {
//      iterations++;
//      SmartDashboard::PutNumber("OrientRobot Iterations ", iterations);
        drive.TankDrive(0, 0);
        brakingFlag = false;
//      difference = angle - ahrs->GetYaw();
//      if (difference > 180)
//        difference = difference - 360;
//      else if (difference < -180)
//	     difference = difference + 360;
//       SmartDashboard::PutNumber("OrientRobot Iteration difference ", difference);
//       if ((abs(difference) > 1.5) && (iterations < 1))
//        return NOTDONEYET;
//      else {
        FirstCallFlag = true;
        return DONE;
//      }
    }
  } // end of braking
  return NOTDONEYET;
}


int Robot::OrientRobotOld (float angle)
{
	//rotate to desired angle
	static bool FirstCallFlag = true; // FirstCallFlag should always be set to true when returning DONE
  static int iterations = 0;
  static bool brakingFlag = false;
  static double brakeStartTime; 
  static float brakeDirection;
  static float firstDifference;

  float difference, speed;

  if (FirstCallFlag) {
    iterations = 0;
    brakingFlag = false;
  }

  if (brakingFlag) {
     if ((AutoTimer.Get() - brakeStartTime) < .2) {
       SmartDashboard::PutNumber("OrientBrake Left Side ", 0.5 * (AUTOFORWARD) * brakeDirection);
       SmartDashboard::PutNumber("OrientBrake Right Side ", -0.5 * (AUTOFORWARD) * brakeDirection);
      drive.TankDrive(0.6 * (AUTOFORWARD) * brakeDirection, -0.6 * (AUTOFORWARD) * brakeDirection);
      return NOTDONEYET;
     }
     else {
       iterations++;
       SmartDashboard::PutNumber("Iterations ", iterations);
       drive.TankDrive(0, 0);
       brakingFlag = false;
       difference = angle - ahrs->GetYaw();
       if (difference > 180)
         difference = difference - 360;
       else if (difference < -180)
		     difference = difference + 360;
SmartDashboard::PutNumber("Iteration difference ", difference);
SmartDashboard::PutNumber("Iteration target     ", angle);
SmartDashboard::PutNumber("Iteration actual     ", ahrs->GetYaw());
       if ((abs(difference) > 1.5) && (iterations < 1))
         return NOTDONEYET;
       else {
         FirstCallFlag = true;
         return DONE;
       } 
     }
  } else {
    // This is the normal path - called when we're not braking at the end of a turn
  	difference = angle - ahrs->GetYaw();
    if (difference > 180)
      difference = difference - 360;
    else if (difference < -180)
	    difference = difference + 360;
    
    if (FirstCallFlag == true) {
      firstDifference = difference;
   	SmartDashboard::PutNumber("OrientRobot Firstdifference", difference);
   	SmartDashboard::PutNumber("OrientRobot First angle", ahrs->GetYaw());
   	SmartDashboard::PutNumber("OrientRobot target angle", angle);
      FirstCallFlag = false;
    }

   	SmartDashboard::PutNumber("OrientRobot difference", difference);
	  //DisplayNAVX();
    SmartDashboard::PutNumber(  "Current Angle", ahrs->GetYaw());
    if ((abs(firstDifference) > 60) &&
        (abs(firstDifference - difference) > 15) &&
        (abs(difference) > 45)) {
           // speed = 1.0;
           speed = 0.65;
    } else {
           // speed = 0.75;
           speed = 0.5;
    }
#ifdef _TESTROBOT_
               speed = 1.0;
#endif
    if (difference > 0) {
       SmartDashboard::PutNumber("OrientRobot Left Side ", speed * (AUTOFORWARD));
       SmartDashboard::PutNumber("OrientRobot Right Side ", -1.0 * speed * (AUTOFORWARD));
       drive.TankDrive(speed * (AUTOFORWARD), -1.0 * speed * (AUTOFORWARD));
       brakeDirection = -1.0;
    } else {
       SmartDashboard::PutNumber("OrientRobot Left Side ", -1.0 * speed * (AUTOFORWARD));
       SmartDashboard::PutNumber("OrientRobot Right Side ", speed * (AUTOFORWARD));
	   	drive.TankDrive(-1.0 * speed * (AUTOFORWARD), speed * (AUTOFORWARD));                       
      brakeDirection = 1.0;
    }

    difference = angle - ahrs->GetYaw();
    if (difference > 180)
      difference = difference - 360;
    else if (difference < -180)
	    difference = difference + 360;
   	SmartDashboard::PutNumber("OrientRobot difference", difference);

    // once we get close, brake, to avoid overshooting our target angle, then check again
    // limit the number of times we do this in case it gets in a loop where it keeps going
    // too far

    if ((abs(difference) <= 6) && (abs(firstDifference - difference) > 15)) {
	   	SmartDashboard::PutNumber("start braking at diff ", difference);
	   	SmartDashboard::PutNumber("start braking at dist ", ahrs->GetYaw());
      brakingFlag = true;
    SmartDashboard::PutNumber("Braking Flag ", 1);
      brakeStartTime = AutoTimer.Get();
      drive.TankDrive(0.75 * (AUTOFORWARD) * brakeDirection, -0.75 * (AUTOFORWARD) * brakeDirection);
    }
  }
  return NOTDONEYET;
}

int Robot::DistanceDrive (float speed, float distance, float angle, bool brake)
{
	static bool FirstCallFlag = true; // FirstCallFlag should always be set to true when returning DONE
	static float autoStartSpeed;
  static float direction;
	static double lastDistance, speedUpDistance, slowDownDistance;
  static int sameCounter;
  static bool brakingFlag;
  static double brakeStartTime; 

	float curve;
	float correction;
	float newSpeed;
	double curDistance;
  double difference;

  if (FirstCallFlag) {
    // Setup distance drive on first call
    // Set initial values for static variables
    brakingFlag = false;
    FirstCallFlag = false;
    if (speed < 0) {
      direction = -1;
    } else {
      direction = 1;
    }
    autoStartSpeed = direction * AUTOSTARTSPEED;
    if (distance < (DRIVERAMPUPDISTANCE * 2)) {
	    speedUpDistance = distance / 2;
	    slowDownDistance = speedUpDistance;
    } else {
	    speedUpDistance = DRIVERAMPUPDISTANCE;
     	slowDownDistance = distance - DRIVERAMPUPDISTANCE;
    }
	  SmartDashboard::PutNumber(  "DistanceDrive Distance", distance);
	  SmartDashboard::PutNumber(  "DistanceDrive Initial angle", ahrs->GetYaw());
  	lastDistance = 0;
    sameCounter = 0;
    leftDriveEncoder.Reset();
  }

 	if (brakingFlag) {
     // Braking flag gets set once we reach targe distance if the brake parameter
     // was specified. Drive in reverse direction at low speed for short duration.
    if ((AutoTimer.Get() - brakeStartTime) < .2) {
    	drive.TankDrive(-0.2 * direction *FORWARD, -0.2 * direction * FORWARD);
      return NOTDONEYET;
    } else {
      drive.TankDrive(0, 0);
      brakingFlag = false;
      FirstCallFlag = true;
      return DONE;
    }
	}

  curve = 0;
  difference = angle - ahrs->GetYaw();
	SmartDashboard::PutNumber(  "DistanceDrive cur Angle", ahrs->GetYaw());
  if (difference > 180)
    difference = difference - 360;
  else if (difference < -180)
	  difference = difference + 360;
  correction = (difference/45) + AUTOCURVECOMP;
  
	curDistance = abs(leftDriveEncoder.GetDistance());

	if (curDistance == lastDistance) {
		if (sameCounter++ == 50) {
				return ERROR;
		}
	} else {
		sameCounter = 0;
		lastDistance = curDistance;
	}

	if (curDistance < speedUpDistance) {
		newSpeed = autoStartSpeed + ((speed - autoStartSpeed) * curDistance)/DRIVERAMPUPDISTANCE;
	} else if ((curDistance > slowDownDistance) && (brake == true)) {
		newSpeed = speed * (distance-curDistance)/DRIVERAMPUPDISTANCE;
	} else {
		newSpeed = speed;
	}

	drive.CurvatureDrive(newSpeed * (AUTOFORWARD), (direction * (curve+correction)), false);
//	drive.CurvatureDrive(newSpeed * (AUTOFORWARD), 0, false);
	curDistance = abs(leftDriveEncoder.GetDistance());
  if (curDistance < distance) {
    return NOTDONEYET;
  } else {
    if (brake) {
      brakingFlag = true;
      brakeStartTime = AutoTimer.Get();
      return NOTDONEYET;
    } else {
      FirstCallFlag = true;
      drive.TankDrive(0, 0);
      return DONE;
    }
  }
  
  // should never get here
  drive.TankDrive(0, 0);
  FirstCallFlag = true;
  return DONE;
}

int Robot::AutonomousShoot(){
	static bool FirstCallFlag = true; // FirstCallFlag should always be set to true when returning DONE 
  static double shootStartTime;
  int rc = NOTDONEYET;

  if (FirstCallFlag == true) {
    // Setup distance drive on first call
    // Set initial values for static variables
    shootStartTime = AutoTimer.Get();
    FirstCallFlag = false;
  }
      // Shoot until done - start with amount of time, to be replaced
      // with sensor-based logic if time allows
      ShootPosMotor.Set(ShootPosMotor.kOff);
      BeltZ1.Set(BELTz1SPD);
      BeltZ2.Set(BELTz2SPDs);
      BeltZ3.Set(-1 * BELTz3SPD);
      //if (AftBallSensor.Get() != LastAftSensorState){
      //  LastAftSensorState = AftBallSensor.Get();
      //  fBalls --;
      //}
      if ((AutoTimer.Get() - shootStartTime) >= AUTOSHOOTTIME) {
        ShootPosMotor.Set(ShootPosMotor.kOff);
        ShootWhl.Set(0);
        BeltZ1.Set(0);
        BeltZ2.Set(0);
        BeltZ3.Set(0);
        //bottomTension.Set(bottomTension.kForward);
        topTension.Set(topTension.kReverse);
        fBalls = 0;
        FirstCallFlag = true;
      rc = DONE;
  }
  return rc;
 }

int Robot::AutonomousSpinUp(double distance){
	static bool FirstCallFlag = true; // FirstCallFlag should always be set to true when returning DONE
  static double speedUpStartTime; 
  static double hoodHeight;
  static int spinUpRC, setHoodRC;
  int rc = NOTDONEYET;

  if (FirstCallFlag) {
    // Setup distance drive on first call
    // Set initial values for static variables
    speedUpStartTime = AutoTimer.Get();
    FirstCallFlag = false;
    spinUpRC = NOTDONEYET;
    setHoodRC = NOTDONEYET;

    // use distance to determine where to set the height of the hood
    //HoodHeight = ?
  }
      // Set hood to correct position and ramp up shooting motor
      if (setHoodRC == NOTDONEYET) {
//        setHoodRC = setHood(hoodHeight);
        if ((AutoTimer.Get() - speedUpStartTime) >= distance) {
          ShootPosMotor.Set(ShootPosMotor.kOff);
          setHoodRC = DONE;
        } else {
          // raise hood
          ShootPosMotor.Set(ShootPosMotor.kReverse);
        }
      }
      if (spinUpRC == NOTDONEYET) {
        ShootWhl.Set(AUTOSHOOTSPEED);
        BeltZ1.Set(0);
        BeltZ2.Set(0);
        BeltZ3.Set(0);
        bottomTension.Set(bottomTension.kReverse); //TEMPORARY
        topTension.Set(topTension.kForward); //both tensioners engaged
        // LastAftSensorState = AftBallSensor.Get();
        if ((AutoTimer.Get() - speedUpStartTime) >= SHOOTSPINUP) {
          spinUpRC = DONE;
        }
      }
      if ((setHoodRC == DONE) && (spinUpRC == DONE)) {
        FirstCallFlag = true;
        rc = DONE;
      }
  return rc;
 }
int Robot::setHood(double time){
  return DONE;
}

int Robot::StartCollector(){
	static bool FirstCallFlag = true; // FirstCallFlag should always be set to true when returning DONE
  static double StartTime; 
  int rc = NOTDONEYET;

  if (FirstCallFlag) {
    StartTime = AutoTimer.Get();
    FirstCallFlag = false;
    pickupM.Set(1);
    pickupPneumatic.Set(pickupPneumatic.kForward);
    BeltZ1.Set(BELTz1SPD);
    BeltZ2.Set(BELTz2SPDp);
    BeltZ3.Set(0);
    topTension.Set(topTension.kReverse);
    bottomTension.Set(bottomTension.kForward);
    // LastForeSensorState = ForeBallSensor.Get();
    // LastAftSensorState = AftBallSensor.Get();
  }

  if ((AutoTimer.Get() - StartTime) >= AUTOPICKUPSTARTTIME) {
    return DONE;
  return NOTDONEYET; 
}
}
int Robot::StopCollector(){
  pickupPneumatic.Set(pickupPneumatic.kReverse);
  pickupM.Set(0);
  BeltZ1.Set(0);
  BeltZ2.Set(0);
  BeltZ3.Set(0);
  topTension.Set(topTension.kReverse);
  bottomTension.Set(bottomTension.kForward);
  return DONE;
}

//------------------------------------------------------------------------
// OrientRobotSlow rotates the robot in place until it is oriented to the 
// specified angle relative to the field (its starting position is 0 degrees).
// OrientRobotSlow is intended to be called when spinning up the shooting motor 
// because the shooter draws power - different tuning is needed to stop braking 
// at the right time, and we also have the luxury of spending a little time 
// trying to get to a close tolerance, knowing that it takes a couple seconds 
// to spin up the shooting motor.
// Specify a value from -180 to +180 where 0 degrees aims straight away from the
// player station, +90 to the right of the player station, -90 to the left of player
// station and -180 or +180 aims back towards the player station.
//-------------------------------------------------------------------------
int Robot::OrientRobotSlow (float angle)
{
  //rotate to desired angle
  static bool FirstCallFlag = true; // FirstCallFlag should always be set to true when returning DONE
  static int iterations = 0;
  static bool brakingFlag = false;
  static double brakeStartTime;
  static double orientStartTime;
  static float direction = 0;
  static float brakeDirection = 0;
  static float firstMagnitude;
  static float leftBrakeSpeed, rightBrakeSpeed;
  static float lastAngle;
  float  magnitude, difference, speed, leftSpeed, rightSpeed;
  bool   changeDirection = false;

#define SLOWMINANGLE 45
#define SLOWMAXSPEED 0.75
#define SLOWMINSPEED 0.5
#define SLOWBRAKEFACTOR 15
//#define ORIENTBRAKESTOP (firstMagnitude/ORIENTBRAKEFACTOR)
// force us to keep rotating until within 3 degrees for first test
#define SLOWBRAKESTOP 6
#define SLOWBRAKESTOPITERATE 1.5

  if (FirstCallFlag) {
    iterations = 0;
    brakingFlag = false;
    direction = 0;
  }

  if (brakingFlag == false) {
    // This is the normal path - called when we're not braking at the end of a turn

    SmartDashboard::PutNumber("OR NRM current angle ", ahrs->GetYaw());
    //-----------------------------------------------------------------------------------
    // calculate direction and magnitude of remaining turn
    // recalculate direction each time in case robot rotates past target before
    // we catch it. We should start braking early enough that this should never happen,
    // but if it does, we need to change direction
    //-----------------------------------------------------------------------------------
    difference = angle - ahrs->GetYaw();
    if (difference > 180)
      difference = difference - 360;
    else if (difference < -180)
      difference = difference + 360;
    if (difference > 0) {
      if (direction != 1) {
        // direction changed - need to recalculate braking speed
        direction = 1;
        brakeDirection = -1;
        changeDirection = true;
      }
      magnitude = difference;
    } else {
      if (direction != 1) {
        // direction changed - need to recalculate braking speed
        direction = -1;
        brakeDirection = 1;
        changeDirection = true;
      }
      magnitude = -1 * difference;
    }
    SmartDashboard::PutNumber("OR NRM difference", difference);
    SmartDashboard::PutNumber("OR NRM magnitude", magnitude);
	
    //-----------------------------------------------------------------------------------
    // once we get close, brake, to avoid overshooting our target angle, then check again
    // limit the number of times we do this in case it gets in a loop where it keeps going
    // too far
    //-----------------------------------------------------------------------------------
    if (((iterations == 0) && (magnitude <= SLOWBRAKESTOP)) || 
        ((iterations > 0) && (magnitude <= SLOWBRAKESTOPITERATE))) {
      brakingFlag = true;
      brakeStartTime = AutoTimer.Get();
      lastAngle = ahrs->GetYaw();
      drive.TankDrive(leftBrakeSpeed, rightBrakeSpeed);
      return NOTDONEYET;
    }

    //-----------------------------------------------------------------------------------
    // Calculate speed based on remaining turn magnitude
    //-----------------------------------------------------------------------------------
    if (magnitude > SLOWMINANGLE) {
      speed = (magnitude - SLOWMINANGLE) / (180 - SLOWMINANGLE);
      speed = speed * (SLOWMAXSPEED-SLOWMINSPEED);
      speed = speed + SLOWMINSPEED;
    } else {
      speed = SLOWMINSPEED;
    }
    leftSpeed = direction * speed * AUTOFORWARD;
    rightSpeed = -1 * leftSpeed;
	
    drive.TankDrive(leftSpeed, rightSpeed);

    //-----------------------------------------------------------------------------------
    // If this is the first call for a new turn, save initial values for braking
    // If robot rotated past target and changed direction, save updated values
    //-----------------------------------------------------------------------------------
    if ((FirstCallFlag == true) || (changeDirection == true)) {
      firstMagnitude = magnitude;
      leftBrakeSpeed = rightSpeed;
      rightBrakeSpeed = leftSpeed;
      FirstCallFlag = false;
      if (FirstCallFlag == true)
        // Don't reset timer on direction change, only first call
        orientStartTime = AutoTimer.Get();
    }

  } else {
    //-----------------------------------------------------------------------------------
    // Braking
    // Compare current angle to last angle to see if robot has reversed direction from
    // direction of rotation
    //-----------------------------------------------------------------------------------
    difference = ahrs->GetYaw() - lastAngle;
    if (difference > 180)
      difference = difference - 360;
    else if (difference < -180)
      difference = difference + 360;
    SmartDashboard::PutNumber("OR BRK difference", difference);
    //-----------------------------------------------------------------------------------
    // Positive difference means we've rotated clockwise since braking
    // Negative difference means we've rotated counterclockwise since braking
    //-----------------------------------------------------------------------------------
    if (((difference >= 0) && (direction == 1)) ||
        ((difference <= 0) && (direction == -1))) {
      //---------------------------------------------------------------------------------
      // We are still rotating in direction of pre-braking rotation
      //---------------------------------------------------------------------------------
      lastAngle = ahrs->GetYaw();
      drive.TankDrive(leftBrakeSpeed, rightBrakeSpeed);
      return NOTDONEYET;
    }
    else {
      iterations++;
      drive.TankDrive(0, 0);
      brakingFlag = false;
      changeDirection = true;        // Force reset of braking speed 

      difference = angle - ahrs->GetYaw();
      if (difference > 180)
        difference = difference - 360;
      else if (difference < -180)
	     difference = difference + 360;

      if (difference > 0) {
        direction = 1;
        brakeDirection = -1;
        magnitude = difference;
      } else {
        // direction changed - need to recalculate braking speed
        direction = -1;
        brakeDirection = 1;
        magnitude = -1 * difference;
      }

      if ((magnitude > 1.5) && (iterations < 1))
         return NOTDONEYET;
      else {
        FirstCallFlag = true;
        return DONE;
      }
    }
  } // end of braking
  return NOTDONEYET;
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
