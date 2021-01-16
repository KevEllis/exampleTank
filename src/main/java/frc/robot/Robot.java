// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically it contains the code
 * necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  //private DifferentialDrive m_myRobot;
  //private XboxController m_leftStick;
  //private XboxController m_rightStick;
  XboxController control = new XboxController(0);
  //motors
  final TalonSRX leftMotor = new TalonSRX(3);
  final TalonSRX rightMotor = new TalonSRX(4);
  @Override
  public void robotInit() {
    //m_myRobot = new DifferentialDrive(new Talon(4), new Talon(3));
    //m_leftStick = new XboxController(0);
    //m_rightStick = new XboxController(1);
  }

  @Override
  public void teleopPeriodic() {
    //m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
    leftMotor.set(ControlMode.PercentOutput, -control.getRawAxis(1));
    rightMotor.set(ControlMode.PercentOutput, control.getRawAxis(5));
  }
}
