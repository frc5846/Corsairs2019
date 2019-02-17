/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AimTarget;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {
 
  NetworkTable net_table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx_entry = net_table.getEntry("tx"); //Horizontal offset from crosshair to target (-27 degrees to 27 degrees)
  NetworkTableEntry ty_entry = net_table.getEntry("ty"); //Vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees)
  NetworkTableEntry ta_entry = net_table.getEntry("ta"); //Target area (0% of image to 100% of image)
  NetworkTableEntry tv_entry = net_table.getEntry("tv"); //Whether the limelight has any valid targets (0 or 1)
  
  
  double tx = tx_entry.getDouble(0.0);
  double ty = ty_entry.getDouble(0.0);
  double ta = ta_entry.getDouble(0.0);
  double tv = tv_entry.getDouble(0.0);

  public double getTV(){
    return tv_entry.getDouble(0.0);
  }

  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new AimTarget());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void visionInit() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //Sets cam to vision processor
  }

  public void driverToggle() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1); //Sets cam to driver camera (disables vision processing)
  }

  public static double countDistance(double angle){
		double baseAngleRad = Math.toRadians(angle + RobotMap.camera_angle); // Convert total camera angle to radians
		double baseAngleTangent = Math.tan(baseAngleRad); // Take the tangent of total angle
		double returnValue = RobotMap.height_difference / baseAngleTangent; // Divide to calculate distance
		return returnValue; // Returns current distance in inches
	}

  public void aimTarget() {
    double left_command = 0.0;
    double right_command = 0.0;
    double steering_adjust = 0.0;
    double tv = tv_entry.getDouble(0.0);
    double tx = tx_entry.getDouble(0.0);
    
    if (tv == 1.0) {
      double heading_error = -tx_entry.getDouble(0.0); //This negative is in the documentation. Figure this out
      
      if (tx > 1.0) {
        steering_adjust = RobotMap.kP*heading_error - RobotMap.minimum_speed;
      }
      
      else if (tx < 1.0) {
        steering_adjust = RobotMap.kP*heading_error + RobotMap.minimum_speed;
      }
      
      SmartDashboard.putNumber("Seek Steering adjust=", steering_adjust);

      // steering_adjust = RobotMap.kP * heading_error;
      left_command += steering_adjust;
      right_command -= steering_adjust;
    }

    Robot.m_dt.tank_drive(left_command, right_command);
  }

  public void seekTarget() {
    double left_command = 0.0;
    double right_command = 0.0;
    double driving_adjust = 0.0;
    double steering_adjust = 0.0;
    double tv = tv_entry.getDouble(0.0);
    double ty = ty_entry.getDouble(0.0);
    double tx = tx_entry.getDouble(0.0);
    
    if (tv == 1.0) {
      double heading_error = -tx;
      double distance_error = -ty;

      if (tx > 1.0) {
        steering_adjust = RobotMap.kPDistanceX*heading_error - RobotMap.minimum_speed;
      }

      else if (tx < 1.0) {
        steering_adjust = RobotMap.kPDistanceX*heading_error + RobotMap.minimum_speed;
      }

      SmartDashboard.putNumber("Seek Steering adjust=", steering_adjust);

      // double current_distance = countDistance(ty);
      // double distance_error = RobotMap.target_distance - current_distance;
      driving_adjust = RobotMap.kPDistanceY * distance_error;
      
      left_command += 1 * (steering_adjust - driving_adjust);
      right_command -= 1 * (steering_adjust + driving_adjust);
    }

    Robot.m_dt.tank_drive(left_command, right_command);


  }

  public void displayTargetData() {
    SmartDashboard.putNumber("Limelight TX", tx_entry.getDouble(0.0));
    SmartDashboard.putNumber("Limelight TY", ty_entry.getDouble(0.0));
    SmartDashboard.putNumber("Limelight TV", tv_entry.getDouble(0.0));
    SmartDashboard.putNumber("Limelight TA", ta_entry.getDouble(0.0));
    
  }

  public void setledON() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  public void setledOFF() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

}
