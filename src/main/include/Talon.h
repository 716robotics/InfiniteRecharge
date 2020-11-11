 #pragma once
  
 #include "frc/PWMSpeedController.h"
  
 namespace frc {
  
 class Talon : public PWMSpeedController {
  public:
   explicit Talon(int channel);
  
   Talon(Talon&&) = default;
   Talon& operator=(Talon&&) = default;
 };
  
 }  // namespace frc