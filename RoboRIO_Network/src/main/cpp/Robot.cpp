// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() 
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //Get the default instance of NetworkTables that was created automatically
  auto inst = nt::NetworkTableInstance::GetDefault();

  m_right.SetInverted(true);

  //Get the table within that instance that contains the data. There can
  //be as many tables as you like and exist to make it easier to organize
  //your data. In this case, it's a table called datatable.
  m_table = inst.GetTable("datatable");

  //Get the entries within that table that correspond to the X and Y values
  //for some operation in your program.
  m_angle = m_table->GetEntry("Angle");
  m_fromCenter = m_table->GetEntry("FromCenter");
  m_centerX = m_table->GetEntry("CenterX");
  m_contourCols = m_table->GetEntry("ContourCols");
}

void Robot::Turn(double angle)
{
  //Get currentAngle and targetAngle
  double currentAngle = (double)m_imu.GetAngle();
  double targetAngle = currentAngle + angle;
  //Deadzone
  double low = targetAngle -0.5;
  double high = targetAngle +0.5;

  while(currentAngle > high || currentAngle < low)
  {
    frc::SmartDashboard::PutNumber("ADI", (double)m_imu.GetAngle());
    frc::SmartDashboard::PutNumber("TurnAngle", angle);
    frc::SmartDashboard::PutNumber("CurrentAngle", currentAngle);

    // If we are above high we need to move right
    if(currentAngle > high)
    {
      frc::SmartDashboard::GetBoolean("isTurningLeft", false);
      m_left.Set(0.1);
      m_right.Set(-0.1);
    }
    // If we below low we need to move left
    else if(currentAngle < low)
    {
      frc::SmartDashboard::GetBoolean("isTurningLeft", true);
      m_left.Set(-0.1);
      m_right.Set(0.1);
    }
    //Reset currentAngle
    currentAngle = (double)m_imu.GetAngle();
  }

  //Once done stop
  m_left.Set(0.0);
  m_right.Set(0.0);
}


/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{
  
  // These comments are a networktable function called AddEntryListener
  // Calls things when a networktable entry changes
  // Not used because not fully necessary and had issues with angle implimentation
  // However these X and Y listeners do work if the Networktable entry "X" is added

  // m_table->AddEntryListener("X",[] (nt::NetworkTable* table, std::string_view name,
  //     nt::NetworkTableEntry entry, std::shared_ptr<nt::Value> value, int flags) {
  //   // PRINT SOMETHING
  //   frc::SmartDashboard::PutNumber("xEntryData", value->GetDouble());
  // }, NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);

  // m_table->AddEntryListener("Y",[] (nt::NetworkTable* table, std::string_view name,
  //     nt::NetworkTableEntry entry, std::shared_ptr<nt::Value> value, int flags) {
  //   // PRINT SOMETHING
  //   frc::SmartDashboard::PutNumber("yEntryData", value->GetDouble());
  // }, NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);


  // Output the data we are getting from the networktables
  // -666 is used as a default because 0 and -1 can sometimes be valed values
  // -666 is too precise and well known that when this is seen it can be likely to assume nothing was grabed
  frc::SmartDashboard::PutNumber("AngleNetwork", m_angle.GetDouble(-666));
  frc::SmartDashboard::PutNumber("fromCenter", m_fromCenter.GetDouble(-666));
  frc::SmartDashboard::PutNumber("CenterX", m_centerX.GetDouble(-666));
  frc::SmartDashboard::PutNumber("ContourCols", m_contourCols.GetDouble(-666));

  // Other outputs
  frc::SmartDashboard::PutNumber("ADI", (double)m_imu.GetAngle());
  frc::SmartDashboard::PutBoolean("A Pressed", controller.GetAButton());

  // Drive
  m_left.Set(controller.GetLeftY());
  m_right.Set(controller.GetRightY());

  // Check if the A button is pressed
  if(controller.GetAButton())
  {
    //Make sure that the angle grabbed is not the same as the previous angle
    if(m_angle.GetDouble(-666) != m_previousAngle)
    {
      //Get the angle from networktables
      m_previousAngle = m_angle.GetDouble(-666);
      //Turn
      if(m_previousAngle != -666)
        Turn(m_previousAngle);
    }
    else
    {
      //Stop
      m_left.Set(0.0);
      m_right.Set(0.0);
    }
  }
}

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
  fmt::print("Auto selected: {}\n", m_autoSelected);

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

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
