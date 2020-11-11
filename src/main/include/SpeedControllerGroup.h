 #pragma once
  
 #include <functional>
 #include <vector>
  
 #include "frc/SpeedController.h"
 #include "frc/smartdashboard/Sendable.h"
 #include "frc/smartdashboard/SendableHelper.h"
  
 namespace frc {
  
 class SpeedControllerGroup : public Sendable,
                              public SpeedController,
                              public SendableHelper<SpeedControllerGroup> {
  public:
   template <class... SpeedControllers>
   explicit SpeedControllerGroup(SpeedController& speedController,
                                 SpeedControllers&... speedControllers);
   ~SpeedControllerGroup() override = default;
  
   SpeedControllerGroup(SpeedControllerGroup&&) = default;
   SpeedControllerGroup& operator=(SpeedControllerGroup&&) = default;
  
   void Set(double speed) override;
   double Get() const override;
   void SetInverted(bool isInverted) override;
   bool GetInverted() const override;
   void Disable() override;
   void StopMotor() override;
   void PIDWrite(double output) override;
  
   void InitSendable(SendableBuilder& builder) override;
  
  private:
   bool m_isInverted = false;
   std::vector<std::reference_wrapper<SpeedController>> m_speedControllers;
 };
  
 }  // namespace frc
  
 #include "frc/SpeedControllerGroup.inc"