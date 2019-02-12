/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

<<<<<<< HEAD
  m_wrist->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,0);
  m_wrist->SetSensorPhase(false);

	m_wrist->Config_kF(0, CONST_kF, 10);
	m_wrist->Config_kP(0, CONST_kP, 10);
	m_wrist->Config_kI(0, CONST_kI, 10);
	m_wrist->Config_kD(0, CONST_kD, 10);

  /*m_elevator.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,0);
  m_elevator.SetSensorPhase(false);
=======

m_wrist->SetSensorPhase(true);

		 m_wrist->Config_kF(kPIDLoopIdx, CONST_kF, kTimeoutMs);
		 m_wrist->Config_kP(kPIDLoopIdx, CONST_kP, kTimeoutMs);
		 m_wrist->Config_kI(kPIDLoopIdx, CONST_kI, kTimeoutMs);
		 m_wrist->Config_kD(kPIDLoopIdx, CONST_kF, kTimeoutMs);

#if defined(__linux__)
    frc::CameraServer::GetInstance()->StartAutomaticCapture();
#else
    wpi::errs() << "Vision only available on Linux.\n";
    wpi::errs().flush();
#endif
  
>>>>>>> parent of 0b5c411... Cleaned code and "fixed" encoder issues

	m_elevator.Config_kF(0, CONST_kF, 10);
	m_elevator.Config_kP(0, CONST_kP, 10);
	m_elevator.Config_kI(0, CONST_kI, 10);
	m_elevator.Config_kD(0, CONST_kD, 10);*/
  

#if defined(__linux__)
    frc::CameraServer::GetInstance()->StartAutomaticCapture();
#else
    wpi::errs() << "Vision only available on Linux.\n";
    wpi::errs().flush();
#endif
  
    m_wrist->ConfigPeakOutputForward(.5,Ktimeout);
    m_wrist->ConfigPeakOutputReverse(.5,Ktimeout);



     m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
    frc::SmartDashboard::PutNumber("Set Rotations", 0);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

<<<<<<< HEAD
void Robot::TeleopInit() {

  m_wrist->SetSelectedSensorPosition(0);
}
//--------------------------------------------------------------------------------------------------
//==============================================================================================
//------------------------------------------------------------------------------------------------------
void Robot::TeleopPeriodic() {

  frc::SmartDashboard::PutNumber("Encoder Positiona", m_encoder.GetPosition());
  frc::SmartDashboard::PutNumber("Encoder Velocity", m_encoder.GetVelocity());

  m_drive.DriveCartesian(m_driveStick.GetX(), m_driveStick.GetY(), m_driveStick.GetZ());


  // Setting up lifter
  if(m_driveStick.GetRawButton(4)){
    m_lift1.Set(1);
    m_lift2.Set(-1);
  }
  else if(m_driveStick.GetRawButton(2)){
    m_lift1.Set(-1);
    m_lift2.Set(1);
  }
  else{
    m_lift1.Set(0);
    m_lift2.Set(0);
  }

  //-------------------------------------------------------------------------------------------------------

  //elevator
  //system
  //that
  //does
  //stuff


  //if(m_encoder.GetPosition() >){
=======
void Robot::TeleopInit() {}
//--------------------------------------------------------------------------------------------------
//==============================================================================================
//------------------------------------------------------------------------------------------------------
void Robot::TeleopPeriodic() {

frc::SmartDashboard::PutNumber("Encoder Positiona", m_encoder.GetPosition());
frc::SmartDashboard::PutNumber("Encoder Velocity", m_encoder.GetVelocity());


m_drive.DriveCartesian(m_driveStick.GetX(), m_driveStick.GetY(), m_driveStick.GetZ());


// Setting up lifter

if(m_driveStick.GetRawButton(4)){

m_lifter1.Set(1);
m_lifter2.Srt(-1);
}
else if(m_driveStick.GetRawButton(2)){

m_lifter1.Set(-1);
m_lifter2.Set(1);
}

else{

m_Lifter.Set(0);

}

//-------------------------------------------------------------------------------------------------------

//elevator
//system
//that
//does
//stuff


//if(m_encoder.GetPosition() >){

//m_elevator.Set(0);

//}


if(m_bottomButton.Get())
{
    stateX = true;
}
else{

    stateX = false;

}
if(m_topButton.Get()){

stateY = true;

}
else{

stateY = false;

}
if(m_encoder.GetPosition() <50){
>>>>>>> parent of 0b5c411... Cleaned code and "fixed" encoder issues

  //m_elevator.Set(0);

<<<<<<< HEAD
  //}


  if(m_bottomButton.Get()){
      stateX = true;
  }
  else {
      stateX = false;
  }
  if(m_topButton.Get()){
    stateY = true;
  }
  else {
    stateY = false;
  }
  if(m_encoder.GetPosition() <50){
    if(m_buttonBoard.GetRawButton(4)){
      m_elevator.SetPosition(50);

    }
    else if(m_buttonBoard.GetRawButton(3)){

      m_elevator.Set(-1);

    }

  else{
    m_elevator.Set(0);
  }
  }
  else{m_elevator.Set(0);}
=======

  if(m_buttonBoard.GetRawButton(4)){

    m_elevator.Set(1);

  }
  else if(m_buttonBoard.GetRawButton(3)){
>>>>>>> parent of 0b5c411... Cleaned code and "fixed" encoder issues

    m_elevator.Set(-1);

<<<<<<< HEAD
  /*if (stateX = true && m_buttonBoard.GetRawButton(3)){

  m_elevator.Set(0);

  }

  if (stateY = true && m_buttonBoard.GetRawButton(4)){

  m_elevator.Set(0);

  }*/


  //--------------------------------------------------------------------------------------
  //wrist


  frc::SmartDashboard::PutNumber("wrist Position", m_wrist->GetSelectedSensorPosition());

  if (m_buttonBoard.GetRawButton(5)){
    wristPosition += 950000;
    frc::SmartDashboard::PutNumber("set wrist Position", wristPosition);
    
  }
  else if (m_buttonBoard.GetRawButton(6)){
    wristPosition = 0;
    frc::SmartDashboard::PutNumber("set wrist Position", 0);

  }
  frc::SmartDashboard::PutNumber("Position from the sensor", m_wrist->GetSelectedSensorPosition());
  m_wrist->Set(ControlMode::Position, wristPosition);

  //-----------------------------------------------------------------------
  //shooter

  if(m_driveStick.GetRawButton(5)){

    m_shooter.Set(1);

  }
  else if(m_driveStick.GetRawButton(6)){

    m_shooter.Set(-1);

  }
  else{

    m_shooter.Set(0);

  }
=======
  }

else{

m_elevator.Set(0);

}
}
else{m_elevator.Set(0);}


/*if (stateX = true && m_buttonBoard.GetRawButton(3)){

m_elevator.Set(0);

}

if (stateY = true && m_buttonBoard.GetRawButton(4)){

m_elevator.Set(0);

}*/


//--------------------------------------------------------------------------------------
//wrist

m_wrist->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,0);

frc::SmartDashboard::PutNumber("wrist Position", m_wrist->GetSelectedSensorPosition());



//-----------------------------------------------------------------------
//shooter

if(m_driveStick.GetRawButton(5)){

m_shooter.Set(1);

}
else if(m_driveStick.GetRawButton(6)){

m_shooter.Set(-1);

}
else{

m_shooter.Set(0);

}
>>>>>>> parent of 0b5c411... Cleaned code and "fixed" encoder issues

}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
