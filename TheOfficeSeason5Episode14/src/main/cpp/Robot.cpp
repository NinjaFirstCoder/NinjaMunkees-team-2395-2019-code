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

 
 // cs:UsbCamera m_camera = CameraServer::GetInstance()->StartAutomaticCapture();

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // *************************************
  // wrist setup
  
  wristMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,0);
  wristMotor->SetSensorPhase(false);
  wristMotor->SetInverted(false);


	wristMotor->Config_kP(kPIDLoopIdx, WRIST_kP, kTimeoutMs);
	wristMotor->Config_kI(kPIDLoopIdx, WRIST_kI, kTimeoutMs);
	wristMotor->Config_kD(kPIDLoopIdx, WRIST_kD, kTimeoutMs);
  wristMotor->Config_kF(kPIDLoopIdx, WRIST_kF, kTimeoutMs);
  //wristMotor->ConfigNominalOutputForward( .5); // drop the power down a little 
  //wristMotor->ConfigNominalOutputReverse(-.5);
  wristMotor->ConfigPeakOutputForward(1);
  wristMotor->ConfigPeakOutputReverse(-1);
  


  // *************************************
  // Camera server setup
  //#if defined(__linux__)
      frc::CameraServer::GetInstance()->StartAutomaticCapture();
  /*#else
      wpi::errs() << "Vision only available on Linux.\n";
      wpi::errs().flush();

  #endif*/
  
    // set PID coefficients
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("ElE / P Gain", kP);
    frc::SmartDashboard::PutNumber("ElE / I Gain", kI);
    frc::SmartDashboard::PutNumber("ElE / D Gain", kD);
    frc::SmartDashboard::PutNumber("ElE / I Zone", kIz);
    frc::SmartDashboard::PutNumber("ElE / Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("ElE / Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("ElE / Min Output", kMinOutput);
    frc::SmartDashboard::PutNumber("ElE / Set Rotations", 0);
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
   wristMotor->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs); // reset wrist to zero
  wristSetPosition = 0;
}



void Robot::AutonomousPeriodic() {
LED.Set(-0.99);

driveTrain.DriveCartesian(mainJoystick.GetX(), mainJoystick.GetY(), mainJoystick.GetZ() *-1);
  //RunDriveTrain();
  RunElevator();
  RunWrist();
  RunShooter();


}



void Robot::TeleopInit() {
 


  elePosition = zeroPoint;
  // get pid values from dashboard for the wrist

  double p = frc::SmartDashboard::GetNumber("DB/Slider 0", 0);
  double i = frc::SmartDashboard::GetNumber("DB/Slider 1", 0);
  double d = frc::SmartDashboard::GetNumber("DB/Slider 2", 0);
  double f = frc::SmartDashboard::GetNumber("DB/Slider 3", 0);
  //setNewWristPID(p,i,d,f);
}



void Robot::TeleopPeriodic() {
 LED.Set(0.53);
  frc::SmartDashboard::PutNumber("joystick", elevatorStick.GetY());
  frc::SmartDashboard::PutNumber("Encoder Positiona", elevatorEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Encoder Velocity", elevatorEncoder.GetVelocity());

driveTrain.DriveCartesian(mainJoystick.GetX(), mainJoystick.GetY(), mainJoystick.GetZ() *-1);
  //RunDriveTrain();
  RunElevator();
  //RunLifter();
  RunWrist();
  RunShooter();
  //CollinsPartyPiece();

  /*if(buttonBoard.GetRawButton(1)) {
    double p = frc::SmartDashboard::GetNumber("DB/Slider 0", 0);
    double i = frc::SmartDashboard::GetNumber("DB/Slider 1", 0);
    double d = frc::SmartDashboard::GetNumber("DB/Slider 2", 0);
    double f = frc::SmartDashboard::GetNumber("DB/Slider 3", 0);
    setNewWristPID(p,i,d,f);
  }*/
}



 // ================================= MAIN TELEOP FUNCTIONS ================================================== //

/*****************************************************
 * This function controls the robot drive train.
 */

void Robot::RunDriveTrain() {
  driveTrain.DriveCartesian(mainJoystick.GetX(), mainJoystick.GetY(), mainJoystick.GetZ() *-1);

}



/*****************************************************
 * This function controls the lifer which is used
 * to raise the robot to score the climb
 */
void Robot::RunLifter() {
  /*
lifter1->Set(ControlMode::PercentOutput, 1);
  if(mainJoystick.GetRawButton(4)){ // run the lifer forward 

    lifter1->Set(ControlMode::PercentOutput, 1);

    lifter2->Set(ControlMode::PercentOutput, -1);

  }

  else if(mainJoystick.GetRawButton(2)){ // run the lifer in reverse 

    lifter1->Set(ControlMode::PercentOutput, -1);

    lifter2->Set(ControlMode::PercentOutput, 1);

  }

  else { // dont move the lifter 

    lifter1->Set(ControlMode::PercentOutput, 0);

    lifter2->Set(ControlMode::PercentOutput, 0);

  }

*/

}



/*****************************************************
 * This function controls the wrist which allows
 * the shooter to change angle. It uses PID to 
 * position and does not have any upper or 
 * lower limit switches. 
 */
void Robot::RunWrist() {
  float elevatorActual = elevatorEncoder.GetPosition();

  if(elevatorStick.GetRawButton(3)) {
    wristSetPosition = wrist_high;
  }
  else if(elevatorStick.GetRawButton(2)) {
    wristSetPosition =  wrist_mid;
  }
  else if(elevatorStick.GetRawButton(1)){
    wristSetPosition = wrist_pickup;
  }
  else if(/*elevatorActual > 4 &&*/ buttonBoard.GetRawButton(7)) {
    wristSetPosition = wrist_low;
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
  double tmp = elevatorStick.GetY() * -1;
  float theth;
  float eleActual = elevatorEncoder.GetPosition();

  if(tmp > deadZone){
    theth = ((tmp - deadZone) * (1 / (1 - deadZone)));
  }
  else if(tmp < - deadZone){
    theth = ((tmp + deadZone) * (-1 / (-1 + deadZone)));
  }
  else{
    theth = 0;
  }

  //wristMotor->ConfigNominalOutputForward( 0.1); // drop the power down a little 
  //wristMotor->ConfigNominalOutputReverse(-0.1);
  frc::SmartDashboard::PutNumber("J0ystick y", elevatorStick.GetY());

  //wristMotor->Set(ControlMode::PercentOutput, -elevatorStick.GetY());

  
    
    if(elePosition > 315) {
      elePosition = 315;
    }
    if(elePosition < -5000 ) {
      elePosition = 0;
    }
    elePosition += theth / 2;

  /*if(buttonBoard.GetRawButton(8)) { // lowest position 
    if(wristSetPosition == wrist_low  ) { // check wrist position 
      // do nothing (do not move elevator lower because wrist will be distroyed)
    } else {
      elePosition = 0;
      //elePosition = -4.786 ;// ball ground pickup
    }
  } else if(buttonBoard.GetRawButton(3)) { // next lowest
    /*if(wristSetPosition == wrist_low) { // check wrist position 
      // do nothing (do not move elevator lower because wrist will be distroyed)
    } else {

      elePosition = 2 ;//- zeroPoint; // drop until the limit switch is found

    }*/
   /* elePosition = 53.857; // disk pickup / lowest deposit
  }
  
  else if(buttonBoard.GetRawButton(2)) { // first position that is okay when the wrist is down
    elePosition = 178.112;// level 2 disk drop
  } else if(buttonBoard.GetRawButton(1)) { 
    elePosition = 279.218 ;// level 3 disk drop
  } else if(buttonBoard.GetRawButton(7)) {
    elePosition = 76.336 ;// rocket level 1 ball shoot
  } else if(buttonBoard.GetRawButton(6)) {
    elePosition = 198.109 ;// level 2 rocket ball shoot/ cargo ball shoot
  } else if(buttonBoard.GetRawButton(5)) {
    elePosition = 315.094 ;// level 3 ball shoot;
  }
  else if(buttonBoard.GetRawButton(4)){
    elePosition = 40;

    if(eleActual = 40){
      wristSetPosition = wrist_low;
    }

    if(eleActual = 40 ){//&& wristMotor->GetSelectedSensorPosition() == 0){
      elePosition = 0;
    }
  }*/

  m_pidController.SetReference(elePosition, rev::ControlType::kPosition);

  frc::SmartDashboard::PutNumber("elevator output in percent", theth);
  frc::SmartDashboard::PutNumber("Elevator / Set Position", elePosition);
  frc::SmartDashboard::PutNumber("Elevator / Actual Position", eleActual);
  frc::SmartDashboard::PutNumber("Elevator / zero Point", zeroPoint);
  frc::SmartDashboard::PutNumber("Elevator / t Set Position", elePosition * 4096);
  frc::SmartDashboard::PutNumber("Elevator / t Actual Position", eleActual);
  frc::SmartDashboard::PutNumber("Elevator / t zero Point", zeroPoint * 4096);
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
// ULTILITY FUNCTIONS

void Robot::setNewWristPID(double p, double i, double d, double f){
    wristMotor->Config_kF(kPIDLoopIdx, f, kTimeoutMs);
    wristMotor->Config_kP(kPIDLoopIdx, p, kTimeoutMs);
    wristMotor->Config_kI(kPIDLoopIdx, i, kTimeoutMs);
    wristMotor->Config_kD(kPIDLoopIdx, d, kTimeoutMs);

    //m_pidController.SetP(p);
    //m_pidController.SetI(i);
    //m_pidController.SetD(d);
    //m_pidController.SetFF(f);

    frc::SmartDashboard::PutNumber("Wrist / p", p);
    frc::SmartDashboard::PutNumber("Wrist / i", i);
    frc::SmartDashboard::PutNumber("Wrist / d", d);
    frc::SmartDashboard::PutNumber("Wrist / f", f);
  }



//LEDs

  void Robot::CollinsPartyPiece() {
    // shooter

    /*if(mainJoystick.GetRawButton(5)){

      LED.Set(-0.57);

    }

    else if(mainJoystick.GetRawButton(6)){

      LED.Set(-0.15);

    }*/

    // lifter

    /*else if(mainJoystick.GetRawButton(4)){ 

      LED.Set(-0.07);

    }

    else if(mainJoystick.GetRawButton(2)){ 

      LED.Set(-0.07);

    }*/

    // elevator

    if(elevatorEncoder.GetVelocity() > 1 || elevatorEncoder.GetVelocity() < -1){

      LED.Set(-0.39);

    }

    else if(elevatorEncoder.GetVelocity() < 1 && elevatorEncoder.GetVelocity() > -1){

      LED.Set(0.77);

    }

    // wrist

    /*else if(wristMotor->GetQuadratureVelocity() > 1 || wristMotor->GetQuadratureVelocity() < -1){

      LED.Set(-0.39);

    }

    else if(wristMotor->GetQuadratureVelocity() < 1 && wristMotor->GetQuadratureVelocity() > -1)){

      LED.Set(-0.87);

    }*/

    // When we do nothing

    else {

      LED.Set(-0.99);

  }





  }



/********************************************************************************************
 * This function is run if the robot is put it test mode. Good for testing ideas without 
 * messing with the teleop or autonomous modes. 
 * 
 * it is also called once every 20ms when enabled. 
 */ 
void Robot::TestPeriodic() {}



#ifndef RUNNING_FRC_TESTS

int main() { return frc::StartRobot<Robot>(); }

#endif