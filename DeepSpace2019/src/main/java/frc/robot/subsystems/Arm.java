/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.ArmCmd;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {

  //Lift Motors
  Victor left_lift_mtr = new Victor(RobotMap.left_lift_port);
  Victor right_lift_mtr = new Victor(RobotMap.right_lift_port);

  //Intake Motors
  Victor left_intake_mtr = new Victor(RobotMap.left_intake_port);
  Victor right_intake_mtr = new Victor(RobotMap.right_intake_port);

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArmCmd());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void intake(double intake_speed) {
    left_intake_mtr.set(-intake_speed);
    right_intake_mtr.set(intake_speed);
  }

  public void lift(double lift_speed) {
    left_lift_mtr.set(-lift_speed);
    right_lift_mtr.set(lift_speed);
  }
}
