/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.File;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.TeleopDrive;
import edu.wpi.first.wpilibj.SerialPort;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import edu.wpi.first.wpilibj.Notifier;


/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {


  public enum MtrCtrl {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  }
  

  //Motor Controllers
  CANSparkMax fl_mtr_ctrl = new CANSparkMax(RobotMap.fl_ctrl_port, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax bl_mtr_ctrl = new CANSparkMax(RobotMap.bl_ctrl_port, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax fr_mtr_ctrl = new CANSparkMax(RobotMap.fr_ctrl_port, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax br_mtr_ctrl = new CANSparkMax(RobotMap.br_ctrl_port, CANSparkMaxLowLevel.MotorType.kBrushless);

  CANEncoder fl_encoder;
  CANEncoder bl_encoder;
  CANEncoder fr_encoder;
  CANEncoder br_encoder;

  private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;

  SpeedControllerGroup left_mtrs = new SpeedControllerGroup(fl_mtr_ctrl, bl_mtr_ctrl);
  SpeedControllerGroup right_mtrs = new SpeedControllerGroup(fr_mtr_ctrl, br_mtr_ctrl);

  AHRS gyro = new AHRS(SerialPort.Port.kMXP);

  ArrayList<CANEncoder> encoder_list = new ArrayList<CANEncoder>(4);

//TODO: Check if following variables are correct,
  private static final String k_path_name = "FirstLeftHatch";
  private Notifier m_follower_notifier;
  
  public void initialize() {
    fl_encoder = fl_mtr_ctrl.getEncoder();
    bl_encoder = bl_mtr_ctrl.getEncoder();
    fr_encoder = fr_mtr_ctrl.getEncoder();
    br_encoder = br_mtr_ctrl.getEncoder();

    encoder_list.add(fl_encoder);
    encoder_list.add(bl_encoder);
    encoder_list.add(fr_encoder);
    encoder_list.add(br_encoder);
  
    gyro.reset();


  }
  
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TeleopDrive());
  }

//*********************************************************/
//          Motor Controller functionality
//*********************************************************/

  //Drive Method (Arcade)
  public void arcade_drive(double turn, double forward) {
    turn *= .90;
    left_mtrs.set((-forward) + turn);
    right_mtrs.set(forward + turn);

  }

  //Drive Method (Tank)
  public void tank_drive(double left, double right) {
    left_mtrs.set(-left);
    right_mtrs.set(right);
  }
  //TODO: check how reset affects positions
  //Returns meters traveled by the motor
  public double getFLPosition() {
    //Converted to meters using circumference * rotations by motor
    return fl_encoder.getPosition() * 2 * Math.PI * RobotMap.wheel_radius;
  }

  public double getBLPosition() {
    //Converted to meters using circumference * rotations by motor
    return bl_encoder.getPosition() * 2 * Math.PI * RobotMap.wheel_radius;
  }

  public double getFRPosition() {
    //Converted to meters using circumference * rotations by motor
    return fr_encoder.getPosition() * 2 * Math.PI * RobotMap.wheel_radius;
  }

  public double getBRPosition() {
    //Converted to meters using circumference * rotations by motor
    return br_encoder.getPosition() * 2 * Math.PI * RobotMap.wheel_radius;
  }

  public int Convert_to_Int() {
    return Math.round((float)fl_encoder.getPosition());
  }

  //Returns rpm of the motor converted to m/s
  public double getFLVelocity() {
    return fl_encoder.getVelocity() * RobotMap.wheel_radius;
  }

  public double getBLVelocity() {
    return bl_encoder.getVelocity() * RobotMap.wheel_radius;
  }

  public double getFRVelocity() {
    return fr_encoder.getVelocity() * RobotMap.wheel_radius;
  }

  public double getBRVelocity() {
    return br_encoder.getVelocity() * RobotMap.wheel_radius;
  }
  
  //Gets motor temperature in fahrenheit
  public double getFLTemp() {
    //Convert from celsius to fahrenheit
    return fl_mtr_ctrl.getMotorTemperature()*(9.0/5) + 32;
  }

  public double getBLTemp() {
    return bl_mtr_ctrl.getMotorTemperature()*(9.0/5) + 32;
  }

  public double getFRTemp() {
    return fr_mtr_ctrl.getMotorTemperature()*(9.0/5) + 32;
  }

  public double getBRTemp() {
    return br_mtr_ctrl.getMotorTemperature()*(9.0/5) + 32;
  }




  public boolean hasFault(MtrCtrl location) {
    short status = 0;
    switch (location) {
    case FRONT_LEFT:
      status = fl_mtr_ctrl.getFaults();
      break;
    
    case FRONT_RIGHT:
      status = fr_mtr_ctrl.getFaults();
      break;

    case BACK_LEFT:
      status = bl_mtr_ctrl.getFaults();

    case BACK_RIGHT:
      status = br_mtr_ctrl.getFaults();
    }
    return status != 0;
  }
  // Fault ID Check
  public boolean hasSpecificFault(MtrCtrl location,CANSparkMax.FaultID faultID) {
    boolean status = false;
    switch (location) {
    case FRONT_LEFT:
      status = fl_mtr_ctrl.getFault(faultID);
      SmartDashboard.putBoolean(faultID.toString(), status);
      break;
    
    case FRONT_RIGHT:
      status = fr_mtr_ctrl.getFault(faultID);
      SmartDashboard.putBoolean(faultID.toString(), status);
      break;

    case BACK_LEFT:
      status = bl_mtr_ctrl.getFault(faultID);
      SmartDashboard.putBoolean(faultID.toString(), status);
      break;

    case BACK_RIGHT:
      status = br_mtr_ctrl.getFault(faultID);
      SmartDashboard.putBoolean(faultID.toString(), status);
      break;
    }

    return status;
  }


//*********************************************************/
//          Pathfinder functionality
//*********************************************************/
public void pathSetup(String pathName) {
  //As of now, bug with Pathweaver, left/right may need to be swapped
  Trajectory left_trajectory = PathfinderFRC.getTrajectory(pathName + ".left");
    Trajectory right_trajectory = PathfinderFRC.getTrajectory(pathName + ".right");

    m_left_follower = new EncoderFollower(left_trajectory);
    m_right_follower = new EncoderFollower(right_trajectory);
    
    // getPosition returns rotation of the motor, therefore convert to count
    int encoderCount = Math.round((float)  fl_encoder.getPosition() * RobotMap.k_ticks_per_rev);
    m_left_follower.configureEncoder(encoderCount, RobotMap.k_ticks_per_rev, RobotMap.wheel_radius * 2);
    // You must tune the PID values on the following line!
    //TODO: change these constants
    m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / RobotMap.k_max_velocity, 0);

    // getPosition returns rotation of the motor, therefore convert to count
    encoderCount = Math.round((float)  fr_encoder.getPosition() * RobotMap.k_ticks_per_rev);

    m_right_follower.configureEncoder(encoderCount, RobotMap.k_ticks_per_rev,
        RobotMap.wheel_radius * 2);
    // You must tune the PID values on the following line!
    //TODO: change these constants
    m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / RobotMap.k_max_velocity, 0);
    
    m_follower_notifier = new Notifier(this::followPath);
    m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
}


public void followPath(){
    int encoderCount = Math.round((float)  fl_encoder.getPosition() * RobotMap.k_ticks_per_rev);
    double left_speed = m_left_follower.calculate(encoderCount);
    encoderCount = Math.round((float)  fr_encoder.getPosition() * RobotMap.k_ticks_per_rev);
    double right_speed = m_right_follower.calculate(encoderCount);
    double heading = gyro.getAngle();
    double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
    double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
    double turn =  0.8 * (-1.0/80.0) * heading_difference;
    fl_mtr_ctrl.set(left_speed + turn);
    fr_mtr_ctrl.set(right_speed - turn);
    
  }
public boolean isPathFinished() {

  if (m_left_follower.isFinished() && m_right_follower.isFinished()) {
    m_follower_notifier.stop();
    return true;
  }
  else {
    return false;
  }
}
  

public void stop (){
  // m_follower_notifier.stop();
  fl_mtr_ctrl.set(0);
  fr_mtr_ctrl.set(0);
}
// public static Waypoint[] test = new Waypoint(x, y, angle)


}
