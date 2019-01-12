/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.TeleopDrive;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  
  //Motor Controllers
  Talon fl_mtr_ctrl = new Talon(RobotMap.fl_ctrl_port);
  Talon bl_mtr_ctrl = new Talon(RobotMap.bl_ctrl_port);
  Talon fr_mtr_ctrl = new Talon(RobotMap.fr_ctrl_port);
  Talon br_mtr_ctrl = new Talon(RobotMap.br_ctrl_port);

  SpeedControllerGroup left_mtrs = new SpeedControllerGroup(fl_mtr_ctrl, bl_mtr_ctrl);
  SpeedControllerGroup right_mtrs = new SpeedControllerGroup(fr_mtr_ctrl, br_mtr_ctrl);

  //Gyro
  public final AHRS gyro = new AHRS(SerialPort.Port.kMXP);

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TeleopDrive());
  }

  //Drive Method (Arcade)
  public void arcade_drive(double turn, double forward) {
    turn *= .90;
    left_mtrs.set((-forward) + turn);
    right_mtrs.set(forward + turn);
  }

  //Drive Method (Tank)
  public void tank_drive(double left, double right) {
    left_mtrs.set(left);
    right_mtrs.set(right);
  }
}
