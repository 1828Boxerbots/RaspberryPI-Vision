// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/EntryListenerFlags.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>

#include <frc/motorcontrol/Victor.h>
#include <frc/ADIS16448_IMU.h>

#include <cameraserver/CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

class Robot : public frc::TimedRobot {
 public:
  void Turn(double angle);

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  // Devices
  frc::Victor m_left{4};
  frc::Victor m_right{3};
  frc::ADIS16448_IMU m_imu{};
  frc::XboxController controller{0};

  //Networktable table
  std::shared_ptr<nt::NetworkTable> m_table;

  //Networktable Entries
  nt::NetworkTableEntry m_angle;
  nt::NetworkTableEntry m_fromCenter;
  nt::NetworkTableEntry m_centerX;
  nt::NetworkTableEntry m_contourCols;

  //The angle at which we must turn to
  double m_previousAngle = 0;
};
