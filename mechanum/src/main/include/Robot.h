/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/WPILib.h"

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/Talon.h>
#include <frc/spark.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

//drive train setup

  frc::Spark m_frontLeft{0};
  frc::Spark m_rearLeft{1};
  frc::Spark m_frontRight{2};
  frc::Spark m_rearRight{3};

  frc::MecanumDrive m_drive{m_frontLeft, m_rearLeft, m_frontRight, m_rearRight};
  frc::Joystick m_driveStick{0};

//lifter setup
  frc::Spark m_Lifter{5};

//elevator setup

  frc::Spark m_elevator{6};
  frc::Joystick m_buttonBoard{1};

//wrist setup

  frc::Spark m_wrist{7};


//shooter setup

  frc::Spark m_shooter{8};

  

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
