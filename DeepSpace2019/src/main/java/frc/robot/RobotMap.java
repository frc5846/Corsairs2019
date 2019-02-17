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
  public static final int RB_Button = 6;
  public static final int LB_Button = 5;
  public static final int Select_Button = 7;
  public static final int Start_Button = 8;

  //Speed Multipliers
  public static double A_Speed = 1.00;
  public static double B_Speed = 0.75;
  public static double Y_Speed = 0.50;
  public static double X_Speed = 0.25;
  public static double lift_speed = 0.5;
  public static double intake_speed = 1.0;

  //Motor Controller Ports
  //CAN
  public static int fl_ctrl_port = 2; 
  public static int bl_ctrl_port = 1;
  public static int fr_ctrl_port = 4;
  public static int br_ctrl_port = 3;
  //PWM
  public static int left_lift_port = 1;
  public static int right_lift_port = 2;
  public static int left_intake_port = 3;
  public static int right_intake_port = 4;
  //Test
  public static int t_fl_ctrl_port = 0;
  public static int t_bl_ctrl_port = 1;
  public static int t_fr_ctrl_port = 2;
  public static int t_br_ctrl_port = 3;

  //Radius of Wheel in Meters 
  public static final double wheel_radius = 0.0762; //(3 inches) may change
 //number of encoder counts per wheel revolution
  public static final int k_ticks_per_rev = 42; //From NEO brushless motor data sheet
  public static final double k_max_velocity = 10; //TODO: change this

  //Vision constants
  //TODO: change these
  public static final double camera_height = 20; //height in inches of camera mounted on the robot
  public static final double kP = .0071; //CHANGE IF TWITCHING LEFT AND RIGHT (increase for more speed/decrease for less)
  public static final double kPDistanceX = .0071; //Same thing as kP make these the same
  public static final double kPDistanceY = .075; //CHANGE IF SPEED APPROACHING TARGET IS TOO FAST/TOO SLOW
  public static final double camera_angle = 0.0;
  public static final double target_height = 22;
  public static final double height_difference = target_height - camera_height;
  public static final double target_distance = 30;
  public static final double navigation_time = 30; //in hundreds of milliseconds 
  public static final double minimum_speed = .1;
}

