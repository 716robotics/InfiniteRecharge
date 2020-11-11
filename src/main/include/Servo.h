#pragma once
  
 #include "frc/PWM.h"
 #include "frc/SpeedController.h"
  
 namespace frc {
  
 class Servo : public PWM {
  public:
   explicit Servo(int channel);
  
   Servo(Servo&&) = default;
   Servo& operator=(Servo&&) = default;
  
   void Set(double value);
  
   void SetOffline();
  
   double Get() const;
  
   void SetAngle(double angle);
  
   double GetAngle() const;
  
   double GetMaxAngle() const;
  
   double GetMinAngle() const;
  
   void InitSendable(SendableBuilder& builder) override;
  
  private:
   double GetServoAngleRange() const;
  
   static constexpr double kMaxServoAngle = 180.0;
   static constexpr double kMinServoAngle = 0.0;
  
   static constexpr double kDefaultMaxServoPWM = 2.4;
   static constexpr double kDefaultMinServoPWM = 0.6;
 };
  
 }  // namespace frc