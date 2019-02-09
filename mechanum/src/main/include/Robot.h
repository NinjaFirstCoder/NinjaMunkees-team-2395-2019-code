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

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/Talon.h>
#include <frc/spark.h>

#include <cameraserver/CameraServer.h>

#include <wpi/raw_ostream.h>

#include "rev/CANSparkMax.h"

#include <frc/DigitalInput.h>
#include <frc/encoder.h>
#include "ctre/Phoenix.h"


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;




//drive train setup-------------------------------------------------

  frc::Spark m_frontLeft{0};
  frc::Spark m_rearLeft{1};
  frc::Spark m_frontRight{2};
  frc::Spark m_rearRight{3};

  frc::MecanumDrive m_drive{m_frontLeft, m_rearLeft, m_frontRight, m_rearRight};
  frc::Joystick m_driveStick{0};

//lifter setup------------------------------------------------

  rev::CANSparkMax m_lift1{2, rev::CANSparkMax::MotorType::kBrushless};
rev::CANEncoder m_lift1E = m_lift1.GetEncoder();

rev::CANSparkMax m_lift2{3, rev::CANSparkMax::MotorType::kBrushless};
rev::CANEncoder m_lift2E = m_lift2.GetEncoder();

//elevator setup---------------------------------------------

rev::CANSparkMax m_elevator{0, rev::CANSparkMax::MotorType::kBrushless};
rev::CANEncoder m_encoder = m_elevator.GetEncoder();

frc::DigitalInput m_bottomButton{0};
frc::DigitalInput m_topButton{1};
  
  frc::Joystick m_buttonBoard{1};

bool stateX;
bool stateY;
//wrist setup------------------------------------------------------------

  TalonSRX * m_wrist = new TalonSRX(1);
  
//shooter setup---------------------------------------------------------------

  frc::Talon m_shooter{8};

  

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  
   
};
