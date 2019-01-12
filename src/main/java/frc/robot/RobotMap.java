/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //Joystick Ports
  public static final int driver_port = 0;
  public static final int manip_port = 1;

  //Axis Mapping
  public static final int axis_lx = 0;
  public static final int axis_ly = 1;
  public static final int axis_rx = 4;
  public static final int axis_ry = 5;

  //Button Mapping
  public static final int A_Button = 1;
  public static final int B_Button = 2;
  public static final int X_Button = 3;
  public static final int Y_Button = 4;

  //Speed Multipliers
  public static double A_Speed = 1.00;
  public static double B_Speed = 0.75;
  public static double Y_Speed = 0.50;
  public static double X_Speed = 0.25;

  //Motor Controller Ports
  public static int fl_ctrl_port = 0;
  public static int bl_ctrl_port = 1;
  public static int fr_ctrl_port = 2;
  public static int br_ctrl_port = 3;

}
