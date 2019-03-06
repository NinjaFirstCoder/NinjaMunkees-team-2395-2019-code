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

#include <frc/Victor.h>

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
  void CollinsPartyPiece();


  // ultil functions
  void setNewWristPID(double p, double i, double d, double f);


  // *************************************
  // input device setup
  frc::Joystick mainJoystick{0};
  frc::Joystick buttonBoard{1};
  frc::Joystick elevatorStick{2};

  // *************************************
  // drive train setup
  frc::Spark frontLeftMotor{0};
  frc::Spark rearLeftMotor{1};
  frc::Spark frontRightMotor{2};
  frc::Spark rearRightMotor{3};

  frc::MecanumDrive driveTrain{frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor};


  // *************************************
  // lifter setup
 VictorSPX * lifter1 = new VictorSPX(5);
 VictorSPX * lifter2 = new VictorSPX(6);

  // *************************************
  // elevator setup
  rev::CANSparkMax elevator{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder elevatorEncoder = elevator.GetEncoder();
  rev::CANPIDController m_pidController = elevator.GetPIDController();

  double kP = 0.45, 
    kI = 0, 
    kD = 0.6, 
    kIz = 0, 
    kFF = 0, 
    kMaxOutput = 1, 
    kMinOutput = -1;


  double elePosition = 0; // measured in rotations 
  double zeroPoint = 0;
  double highLimit = 0;
  float deadZone = .25;

  // *************************************
  // wrist setup
  TalonSRX * wristMotor = new TalonSRX(1);
  double wristSetPosition = 1;

 // wrist PID vars
  const float WRIST_kP = 0.08;
  const float WRIST_kI = 0.1;
  const float WRIST_kD = 0.6;
  const float WRIST_kF = 0;

  const int kPIDLoopIdx = 0;
  const int kTimeoutMs = 10;
  const float  wrist_low = 0;
  const float  wrist_pickup = 0;
  const float  wrist_mid = 4096 / 2;
  const float  wrist_high =  4096;


  // *************************************
  //shooter setup
  frc::Spark shooterMotor{8};

  //88888888888888888888888888888888888888888888888888888888
  //LEDs setup
  frc::Spark LED{5};





 private:

  frc::SendableChooser<std::string> m_chooser;

  const std::string kAutoNameDefault = "Default";

  const std::string kAutoNameCustom = "My Auto";

  std::string m_autoSelected;

};
