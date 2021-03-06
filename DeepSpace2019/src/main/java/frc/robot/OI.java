/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  Joystick driver_js = new Joystick(RobotMap.driver_port); //driver
  Joystick manip_js = new Joystick(RobotMap.manip_port); //manipulator

  //Driver Controller Axis

//*********************************************************/
//          Driver Controller Axis
//*********************************************************/
  public double driver_axis_lx() {
    return driver_js.getRawAxis(RobotMap.axis_lx);
  } 

  public double driver_axis_ly() {
    return driver_js.getRawAxis(RobotMap.axis_ly);
  }

  public double driver_axis_rx() {
    return driver_js.getRawAxis(RobotMap.axis_rx);
  }

  public double driver_axis_ry() {
    return driver_js.getRawAxis(RobotMap.axis_ry);
  }

//*********************************************************/
//          Manipulator Controller Axis
//*********************************************************/  

  public double manip_axis_lx() {
    return manip_js.getRawAxis(RobotMap.axis_lx);
  }

  public double manip_axis_ly() {
    return manip_js.getRawAxis(RobotMap.axis_ly);
  }

  public double manip_axis_rx() {
    return manip_js.getRawAxis(RobotMap.axis_rx);
  }

  public double manip_axis_ry() {
    return manip_js.getRawAxis(RobotMap.axis_ry);
  }

  //*********************************************************/
  //          Driver Controller Buttons
  //*********************************************************/
  public boolean driver_button_A() {
    return driver_js.getRawButton(RobotMap.A_Button);
  }

  public boolean driver_button_B() {
    return driver_js.getRawButton(RobotMap.B_Button);
  }

  public boolean driver_button_X() {
    return driver_js.getRawButton(RobotMap.X_Button);
  }

  public boolean driver_button_Y() {
    return driver_js.getRawButton(RobotMap.Y_Button);
  }

  public boolean driver_button_RB() {
    return driver_js.getRawButton(RobotMap.RB_Button);
  }

  public boolean driver_button_LB() {
    return driver_js.getRawButton(RobotMap.LB_Button);
  }

  public boolean driver_button_select() {
    return driver_js.getRawButton(RobotMap.Select_Button);
  }

  public boolean driver_button_start() {
    return driver_js.getRawButton(RobotMap.Start_Button);
  }

  //*********************************************************/
  //          Manipulator Controller Buttons
  //*********************************************************/
  public boolean manip_button_A() {
    return manip_js.getRawButton(RobotMap.A_Button);
  }

  public boolean manip_button_B() {
    return manip_js.getRawButton(RobotMap.B_Button);
  }

  public boolean manip_button_X() {
    return manip_js.getRawButton(RobotMap.X_Button);
  }

  public boolean manip_button_Y() {
    return manip_js.getRawButton(RobotMap.Y_Button);
  }

  public boolean manip_button_RB() {
    return manip_js.getRawButton(RobotMap.RB_Button);
  }

  public boolean manip_Button_LB() {
    return manip_js.getRawButton(RobotMap.LB_Button);
  }

  //Control System
  public boolean isTank = false;
  
}
