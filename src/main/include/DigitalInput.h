#pragma once
  
 #include "frc/DigitalSource.h"
 #include "frc/smartdashboard/Sendable.h"
 #include "frc/smartdashboard/SendableHelper.h"
  
 namespace frc {
  
 class DigitalGlitchFilter;
 class SendableBuilder;
  
 class DigitalInput : public DigitalSource,
                      public Sendable,
                      public SendableHelper<DigitalInput> {
  public:
   explicit DigitalInput(int channel);
  
   ~DigitalInput() override;
  
   DigitalInput(DigitalInput&&) = default;
   DigitalInput& operator=(DigitalInput&&) = default;
  
   bool Get() const;
  
   // Digital Source Interface
   HAL_Handle GetPortHandleForRouting() const override;
  
   AnalogTriggerType GetAnalogTriggerTypeForRouting() const override;
  
   bool IsAnalogTrigger() const override;
  
   int GetChannel() const override;
  
   void SetSimDevice(HAL_SimDeviceHandle device);
  
   void InitSendable(SendableBuilder& builder) override;
  
  private:
   int m_channel;
   hal::Handle<HAL_DigitalHandle> m_handle;
  
   friend class DigitalGlitchFilter;
 };
  
 }  // namespace frc