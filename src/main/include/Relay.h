#pragma once
  
 #include <memory>
  
 #include <hal/Types.h>
 #include <wpi/raw_ostream.h>
  
 #include "frc/ErrorBase.h"
 #include "frc/MotorSafety.h"
 #include "frc/smartdashboard/Sendable.h"
 #include "frc/smartdashboard/SendableHelper.h"
  
 namespace frc {
  
 class SendableBuilder;
  
 class Relay : public MotorSafety,
               public Sendable,
               public SendableHelper<Relay> {
  public:
   enum Value { kOff, kOn, kForward, kReverse };
   enum Direction { kBothDirections, kForwardOnly, kReverseOnly };
  
   explicit Relay(int channel, Direction direction = kBothDirections);
  
   ~Relay() override;
  
   Relay(Relay&&) = default;
   Relay& operator=(Relay&&) = default;
  
   void Set(Value value);
  
   Value Get() const;
  
   int GetChannel() const;
  
   // MotorSafety interface
   void StopMotor() override;
  
   void GetDescription(wpi::raw_ostream& desc) const override;
  
   void InitSendable(SendableBuilder& builder) override;
  
  private:
   int m_channel;
   Direction m_direction;
  
   hal::Handle<HAL_RelayHandle> m_forwardHandle;
   hal::Handle<HAL_RelayHandle> m_reverseHandle;
 };
  
 }  // namespace frc