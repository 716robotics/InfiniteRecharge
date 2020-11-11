#pragma once

#include <string>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <Talon.h>
#include <XboxController.h>
#include <SpeedControllerGroup.h>
#include <Mappings.h>
#include <DifferentialDrive.h>
#include <ColorSensorV3.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Timer.h>
#include <frc/Compressor.h>
#include <frc/DriverStation.h>
#include "Encoder.h"
#include "DigitalInput.h"
#include "AHRS.h"
#include <Relay.h>
#include <Servo.h>
class Robot : public frc::TimedRobot {
  //Input Devices:
  frc::Joystick leftDriveStick{0};
  frc::Joystick rightDriveStick{1};
  frc::XboxController gamepad{2};
  //Drive motors
  frc::Talon lDrive0{PWMCHANNELDRIVEL0};
  frc::Talon lDrive1{PWMCHANNELDRIVEL1};
  frc::Talon rDrive0{PWMCHANNELDRIVER0};
  frc::Talon rDrive1{PWMCHANNELDRIVER1};
  frc::SpeedControllerGroup lDrive{lDrive0, lDrive1};
  frc::SpeedControllerGroup rDrive{rDrive0, rDrive1};
  frc::DifferentialDrive drive{lDrive, rDrive};
  //Effector motors
  frc::Talon cpMotor{PWMCHANNELCPMOTOR};
  frc::Talon climb{PWMCHANNELCLIMBM};
  frc::Servo climbSRVO{PWMCHANNELCLIMBSRVO};
  frc::Talon pickupM{PWMCHANNELPICKUP};
  frc::Talon ShootWhlA{PWMCHANNELSHOOTWHLA};
  frc::Talon ShootWhlB{PWMCHANNELSHOOTWHLB};
  frc::Relay ShootPosMotor{RELAYCHANNELSHOOTPOS, ShootPosMotor.kBothDirections};
  frc::Talon BeltZ1A{PWNCHANNELBELTZ1A};
  frc::Talon BeltZ1B{PWMCHANNELBELTZ1B};
  frc::Talon BeltZ2{PWMCHANNELBELTZ2};
  frc::Talon BeltZ3{PWMCHANNELBELTZ3};
  frc::SpeedControllerGroup ShootWhl{ShootWhlA, ShootWhlB};
  frc::SpeedControllerGroup BeltZ1{BeltZ1A, BeltZ1B};
  //Pneumatics
  frc::Compressor compressor;
  frc::DoubleSolenoid colorSensorArm{PCMARTICULATOROUT, PCMARTICULATORIN};
  frc::DoubleSolenoid pickupPneumatic{PCMPICKUPOUT, PCMPICKUPIN};
  frc::DoubleSolenoid bottomTension{PCMBOTTOMTENSIONF, PCMBOTTOMTENSIONR};
  frc::DoubleSolenoid topTension{PCMTOPTENSIONF, PCMTOPTENSIONR};
  //Onboard Sensors
  static constexpr auto i2cPort0 = frc::I2C::Port::kOnboard;
  static constexpr auto i2cPort1 = frc::I2C::Port::kMXP;
  AHRS *ahrs;
  rev::ColorSensorV3 colorSensor0{i2cPort0};
 
  frc::Encoder ShootPosEncoder{DIOSHOOTPOSA, DIOSHOOTPOSB,false,Encoder::k4X};
	frc::Encoder leftDriveEncoder{DIGCHANNELLEFTDRIVEA,DIGCHANNELLEFTDRIVEB,false,Encoder::k4X};
	frc::Encoder rightDriveEncoder{DIGCHANNELRIGHTDRIVEA,DIGCHANNELRIGHTDRIVEB,false,Encoder::k4X};

  frc::DigitalInput ForeBallSensor{DIOFRONTBALLDETECT};
  frc::DigitalInput AftBallSensor{DIOAFTBALLDETECT};
  
  //Global Variabels
  int fBalls = 0; //number of balls as measured by the fore sensor
  int aBalls = 0; //number of balls as measured by the aft sensor
  frc::Timer PosTimeout;
  frc::Timer RotTimeout;
  frc::Timer RumbleTimer;
  frc::Timer AutoTimer;
  //Flags
  bool doLowPower = false;
  bool doPickupRetract = false;
  bool doConfirmRumble = false;
  bool doFatalRumble = false;
  bool doShootErrRumble = false;
  bool doPickupErrRumble = false;
  bool doCPerrRumble = false;
  bool allowshChange = true;
  bool doClimbErrRumble = false;
  bool sdfr = false;
  bool allowclimboverride = false;
  enum RobotStateType {
    rs_main,
    rs_AutoPos,
    rs_AutoRot,
    rs_manualCPL,
    rs_Shoot,
    rs_pickup,
    rs_climb
  };
  RobotStateType rs = rs_main;
  enum AutoStateType {
    as_initialState,
    as_orientState,
    as_shootState,
    as_pauseState,
    as_enemyState,
    as_trenchState,
    as_trenchState2,
    as_middleState,
    as_moveofflineState,
    as_finalState,
    as_testState
  };
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopMain();
  void TestPeriodic() override;
  void StraightDrive();
  int IdentifyColor();
  void PositionControl();
  void RotationControl();
  void PickUp();
  void RetractPickUp();
  void Shoot();
  void ManualRotation();
  void Climbing();
  void Abort();
  void LowPower();
  void CPerrorRumble();
  void PickupErrorRumble();
  void ShooterErrorRumble();
  void FatalErrorRumble();
  void ClimbErrorRumble();
  void confirmRumble();
  void NavX_Diag();
  int OrientRobot (float);
  int OrientRobotSlow (float);
  int OrientRobotOld (float);
  int DistanceDrive (float, float, float, bool);
  int AutonomousShoot();
  int AutonomousSpinUp(double distance);
  int setHood(double height);
  int StartCollector();
  int StopCollector();
  void testThreadRoutine(int);
 
  bool inThread = false;

 private:
  frc::SendableChooser<std::string> au_StartPos;
  const std::string st_Goal = "Centered On Goal";
  const std::string st_Trench_A = "Aligned With ALLIANCE trench";
  const std::string st_Midfield = "Center of Start Line";
  const std::string st_Test = "Test";
  std::string au_SelectedStartPos;
  frc::SendableChooser<std::string> au_MidStart;
  const std::string trench_cells = "Collect Trench Cells";
  const std::string middle_cells = "Collect Middle Field Cells";
  const std::string off_line = "Move Off Line";
  const std::string vs_field = "Move To Other End";
  std::string au_SelectedMidStart;
  int au_AuToDeLay;
};
