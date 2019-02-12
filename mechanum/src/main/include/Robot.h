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

  // teleop function prototypes
  void RunDriveTrain();
  void RunLifter();
  void RunWrist();
  void RunElevator(); 
  void RunShooter();

  // ultil functions
  void setNewWristPID(double p, double i, double d, double f);


  // *************************************
  // input device setup
  frc::Joystick mainJoystick{0};
  frc::Joystick buttonBoard{1};


  // *************************************
  // drive train setup
  frc::Spark frontLeftMotor{0};
  frc::Spark rearLeftMotor{1};
  frc::Spark frontRightMotor{2};
  frc::Spark rearRightMotor{3};

  frc::MecanumDrive driveTrain{frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor};
  
  // *************************************
  // lifter setup
  rev::CANSparkMax leftLifterMotor{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder leftLifterMotorEncoder = leftLifterMotor.GetEncoder();

  rev::CANSparkMax rightLifterMotor{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder rightLifterMotorEncoder = rightLifterMotor.GetEncoder();

  // *************************************
  // elevator setup
  rev::CANSparkMax elevator{0, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder elevatorEncoder = elevator.GetEncoder();

  frc::DigitalInput elevatorUpperLimitSwitch{0};
  frc::DigitalInput elevatorLowerLimitSwitch{1};
  
  bool stateX;
  bool stateY;

  // *************************************
  // wrist setup
  TalonSRX * wristMotor = new TalonSRX(1);
  double wristSetPosition = 0;

  // wrist PID vars
  const float WRIST_kP = 0.01;
  const float WRIST_kI = 0;
  const float WRIST_kD = 0;
  const float WRIST_kF = 0;

  const int kPIDLoopIdx = 0;
  const int kTimeoutMs = 10;

  // *************************************
  //shooter setup
  frc::Talon shooterMotor{8};

  

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  
   
};