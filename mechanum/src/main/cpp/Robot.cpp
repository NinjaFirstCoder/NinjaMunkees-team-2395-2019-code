/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "Robot.h" // also contains majority of includes 
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>


/*********************************************************************************************
 * This function is called once when the robot is powered on. It is where all of the 
 * sensor/motor controller setup should be. No movement code should live in here. 
 * 
 */
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // *************************************
  // wrist setup
  wristMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,0);
  wristMotor->SetSensorPhase(true);
  wristMotor->SetInverted(false);

	wristMotor->Config_kF(kPIDLoopIdx, WRIST_kF, kTimeoutMs);
	wristMotor->Config_kP(kPIDLoopIdx, WRIST_kP, kTimeoutMs);
	wristMotor->Config_kI(kPIDLoopIdx, WRIST_kI, kTimeoutMs);
	wristMotor->Config_kD(kPIDLoopIdx, WRIST_kD, kTimeoutMs);

  wristMotor->ConfigNominalOutputForward( 0.5); // drop the power down a little 
  wristMotor->ConfigNominalOutputReverse(-0.5);


  // *************************************
  // Camera server setup
  #if defined(__linux__)
      frc::CameraServer::GetInstance()->StartAutomaticCapture();
  #else
      wpi::errs() << "Vision only available on Linux.\n";
      wpi::errs().flush();
  #endif
  


}

/*********************************************************************************************
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 * 
 * DO NOT PUT ANY MOVEMENT CODE IN HERE! ROBOT WILL BE DISQUAILFIED. 
 */
void Robot::RobotPeriodic() {


}

/*******************************************************************************************
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

/********************************************************************************************
 * This function is called every 20ms when the robot is enabled in autonomous mode.
 */
void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}


/*******************************************************************************************
 * This is the first function that is called when the robot is pun into teleop mode
 * and enabled. It is run only once, then the OS begins calling TeleopPeriodic
 */
void Robot::TeleopInit() {
  wristMotor->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs); // reset wrist to zero
}

/*******************************************************************************************
 * This is the function where the majority of the robot code lives. It is called once 
 * every 20ms when the robot is enabled in teleop mode. 
 * 
 * MAKE SURE THIS FUNCTION DOESN'T TAKE MORE THAN 20MS TO RUN!!!
 * so that means: no loops, delays or pauses
 */
void Robot::TeleopPeriodic() {
  // display encoder values 
  frc::SmartDashboard::PutNumber("Encoder Positiona", elevatorEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Encoder Velocity", elevatorEncoder.GetVelocity());

  RunDriveTrain();
  RunElevator();
  RunLifter();
  RunWrist();
  RunShooter();

}


// ================================= MAIN TELEOP FUNCTIONS ================================================== //
/*****************************************************
 * This function controls the robot drive train.
 */
void Robot::RunDriveTrain() {
  driveTrain.DriveCartesian(mainJoystick.GetX(), mainJoystick.GetY(), mainJoystick.GetZ());

}

/*****************************************************
 * This function controls the lifer which is used
 * to raise the robot to score the climb
 */
void Robot::RunLifter() {
  if(mainJoystick.GetRawButton(4)){ // run the lifer forward 
    leftLifterMotor.Set(1);
    rightLifterMotor.Set(-1);
  }
  else if(mainJoystick.GetRawButton(2)){ // run the lifer in reverse 
    leftLifterMotor.Set(-1);
    rightLifterMotor.Set(1);
  }
  else { // dont move the lifter 
    leftLifterMotor.Set(0);
    rightLifterMotor.Set(0);
  }

}

/*****************************************************
 * This function controls the wrist which allows
 * the shooter to change angle. It uses PID to 
 * position and does not have any upper or 
 * lower limit switches. 
 */
void Robot::RunWrist() {
  float rots = 10; // number of rotations for the first point

  if(buttonBoard.GetRawButton(6)) {
    wristSetPosition = rots * 4096;
  }
  else if(buttonBoard.GetRawButton(5)) {
    wristSetPosition = 0;
  }
  frc::SmartDashboard::PutNumber("Wrist / Actual Position", wristMotor->GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("Wrist / Set Position", wristSetPosition);
  wristMotor->Set(ControlMode::Position, wristSetPosition);

}


/*****************************************************
 * This function controls the robots elevator. The
 * elevator raises and lowers the shooter. It has 
 * a neo motor at the bottom with an encoder
 * and has a limit switch at both the upper and lower
 * extent of its travel
 */
void Robot::RunElevator() {
  if(elevatorUpperLimitSwitch.Get()) {
    stateX = true;
  }
  else {
    stateX = false;
  }

  if(elevatorLowerLimitSwitch.Get()) {
    stateY = true;
  }
  else {
    stateY = false;
  }

  if(elevatorEncoder.GetPosition() <50) {
    if(buttonBoard.GetRawButton(4)) {
      elevator.Set(1);
    }
    else if(buttonBoard.GetRawButton(3)) {
      elevator.Set(-1);
    }
    else {
      elevator.Set(0);
    }
  }
  else{elevator.Set(0);}


  /*if (stateX = true && m_buttonBoard.GetRawButton(3)){
    elevator.Set(0);
  }
  if (stateY = true && m_buttonBoard.GetRawButton(4)){
    elevator.Set(0);
  }*/
}


/*****************************************************
 * This function runs the shooter. The shooter has
 * no sensors attached to it whatsoever 
 */
void Robot::RunShooter() {
  if(mainJoystick.GetRawButton(5)){
    shooterMotor.Set(1);
  }
  else if(mainJoystick.GetRawButton(6)){
    shooterMotor.Set(-1);
  }
  else{
    shooterMotor.Set(0);
  }
}

// ================================= END OF MAIN TELEOP FUNCTIONS ================================================== //


/********************************************************************************************
 * This function is run if the robot is put it test mode. Good for testing ideas without 
 * messing with the teleop or autonomous modes. 
 * 
 * it is also called once every 20ms when enabled. 
 */ 
void Robot::TestPeriodic() {


}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif