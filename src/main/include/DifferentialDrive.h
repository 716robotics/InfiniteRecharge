#pragma once
  
 #include <wpi/raw_ostream.h>
  
 #include "frc/drive/RobotDriveBase.h"
 #include "frc/smartdashboard/Sendable.h"
 #include "frc/smartdashboard/SendableHelper.h"
  
 namespace frc {
  
 class SpeedController;
  
 class DifferentialDrive : public RobotDriveBase,
                           public Sendable,
                           public SendableHelper<DifferentialDrive> {
  public:
   static constexpr double kDefaultQuickStopThreshold = 0.2;
   static constexpr double kDefaultQuickStopAlpha = 0.1;
  
   DifferentialDrive(SpeedController& leftMotor, SpeedController& rightMotor);
  
   ~DifferentialDrive() override = default;
  
   DifferentialDrive(DifferentialDrive&&) = default;
   DifferentialDrive& operator=(DifferentialDrive&&) = default;
  
   void ArcadeDrive(double xSpeed, double zRotation, bool squareInputs = true);
  
   void CurvatureDrive(double xSpeed, double zRotation, bool isQuickTurn);
  
   void TankDrive(double leftSpeed, double rightSpeed, bool squareInputs = true);
  
   void SetQuickStopThreshold(double threshold);
  
   void SetQuickStopAlpha(double alpha);
  
   bool IsRightSideInverted() const;
  
   void SetRightSideInverted(bool rightSideInverted);
  
   void StopMotor() override;
   void GetDescription(wpi::raw_ostream& desc) const override;
  
   void InitSendable(SendableBuilder& builder) override;
  
  private:
   SpeedController* m_leftMotor;
   SpeedController* m_rightMotor;
  
   double m_quickStopThreshold = kDefaultQuickStopThreshold;
   double m_quickStopAlpha = kDefaultQuickStopAlpha;
   double m_quickStopAccumulator = 0.0;
   double m_rightSideInvertMultiplier = -1.0;
 };
  
 }  // namespace frc