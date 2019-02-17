/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AimTarget extends Command {
  public AimTarget() {
    requires(Robot.m_vs);
    // requires(Robot.m_dt);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (Robot.m_oi.driver_button_RB() && Robot.m_vs.getTV() == 1.0) {
      Robot.m_vs.aimTarget();
    }

    if (Robot.m_oi.driver_button_LB() && Robot.m_vs.getTV() == 1.0) {
      Robot.m_vs.seekTarget();
    }

    if (Robot.m_oi.driver_button_select()) {
      Robot.m_vs.setledOFF();
    }

    if (Robot.m_oi.driver_button_start()) {
      Robot.m_vs.setledON();
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
