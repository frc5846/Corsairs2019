/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class TeleopDrive extends Command {

  public static double scale = 1.00;
  int direction = 1;

  public TeleopDrive() {
    requires(Robot.m_dt);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    
    if (Robot.m_oi.driver_button_A()) {
      scale = RobotMap.A_Speed;
      direction = 1;
    }

    if (Robot.m_oi.driver_button_B()) {
      scale = RobotMap.B_Speed;
      direction = 1;
    }

    if (Robot.m_oi.driver_button_Y()) {
      scale = RobotMap.Y_Speed;
      direction = 1;
    }

    if (Robot.m_oi.driver_button_X()) {
      scale = RobotMap.X_Speed;
      direction = 1;
    }

    if (Robot.m_oi.isTank) {
      Robot.m_dt.tank_drive(Robot.m_oi.driver_axis_ly() * scale * direction, Robot.m_oi.driver_axis_ry() * scale * direction);
    }

    if (!Robot.m_oi.isTank) {
    Robot.m_dt.arcade_drive(Robot.m_oi.driver_axis_lx() * scale, Robot.m_oi.driver_axis_ly() * scale * direction);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
