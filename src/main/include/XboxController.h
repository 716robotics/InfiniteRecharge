 /*----------------------------------------------------------------------------*/
 /* Copyright (c) 2016-2019 FIRST. All Rights Reserved.                        */
 /* Open Source Software - may be modified and shared by FRC teams. The code   */
 /* must be accompanied by the FIRST BSD license file in the root directory of */
 /* the project.                                                               */
 /*----------------------------------------------------------------------------*/
  
 #pragma once
  
 #include "frc/GenericHID.h"
  
 namespace frc {
  
 class XboxController : public GenericHID {
  public:
   explicit XboxController(int port);
  
   virtual ~XboxController() = default;
  
   XboxController(XboxController&&) = default;
   XboxController& operator=(XboxController&&) = default;
  
   double GetX(JoystickHand hand) const override;
  
   double GetY(JoystickHand hand) const override;
  
   double GetTriggerAxis(JoystickHand hand) const;
  
   bool GetBumper(JoystickHand hand) const;
  
   bool GetBumperPressed(JoystickHand hand);
  
   bool GetBumperReleased(JoystickHand hand);
  
   bool GetStickButton(JoystickHand hand) const;
  
   bool GetStickButtonPressed(JoystickHand hand);
  
   bool GetStickButtonReleased(JoystickHand hand);
  
   bool GetAButton() const;
  
   bool GetAButtonPressed();
  
   bool GetAButtonReleased();
  
   bool GetBButton() const;
  
   bool GetBButtonPressed();
  
   bool GetBButtonReleased();
  
   bool GetXButton() const;
  
   bool GetXButtonPressed();
  
   bool GetXButtonReleased();
  
   bool GetYButton() const;
  
   bool GetYButtonPressed();
  
   bool GetYButtonReleased();
  
   bool GetBackButton() const;
  
   bool GetBackButtonPressed();
  
   bool GetBackButtonReleased();
  
   bool GetStartButton() const;
  
   bool GetStartButtonPressed();
  
   bool GetStartButtonReleased();
  
   enum class Button {
     kBumperLeft = 5,
     kBumperRight = 6,
     kStickLeft = 9,
     kStickRight = 10,
     kA = 1,
     kB = 2,
     kX = 3,
     kY = 4,
     kBack = 7,
     kStart = 8
   };
 };
  
 }  // namespace frc