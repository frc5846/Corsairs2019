/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax.FaultID;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain.MtrCtrl;
import frc.robot.commands.ManualAuto;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;
  public static Drivetrain m_dt = new Drivetrain();
  public static Vision m_vs = new Vision();
  public static Arm m_arm = new Arm();

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<Controls> m_control;

  public enum Controls {
    Arcade,
    Tank,
  }
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_dt.initialize();
    m_control = new SendableChooser<Controls>();
    
    // chooser.addOption("My Auto", new MyAutoCommand());
   // SmartDashboard.putData("Auto mode", m_chooser);
   m_control.addDefault("Arcade", Controls.Arcade);
   m_control.addObject("Tank", Controls.Tank);
   SmartDashboard.putData("Control Scheme", m_control);
  
   CameraServer.getInstance().startAutomaticCapture(1);
   SmartDashboard.putData(Scheduler.getInstance());
   SmartDashboard.putData(m_dt);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //Checking frontleft
    if (Robot.m_dt.hasFault(MtrCtrl.FRONT_LEFT)) {
      Robot.m_dt.hasSpecificFault(MtrCtrl.FRONT_LEFT, FaultID.kBrownout);
      Robot.m_dt.hasSpecificFault(MtrCtrl.FRONT_LEFT, FaultID.kOvercurrent);
      Robot.m_dt.hasSpecificFault(MtrCtrl.FRONT_LEFT, FaultID.kStall);
      Robot.m_dt.hasSpecificFault(MtrCtrl.FRONT_LEFT, FaultID.kMotorFault);
      Robot.m_dt.hasSpecificFault(MtrCtrl.FRONT_LEFT, FaultID.kSensorFault);
      Robot.m_dt.hasSpecificFault(MtrCtrl.FRONT_LEFT, FaultID.kStall);
    }
    if (Robot.m_dt.hasFault(MtrCtrl.FRONT_RIGHT)) {
      Robot.m_dt.hasSpecificFault(MtrCtrl.FRONT_RIGHT, FaultID.kBrownout);
      Robot.m_dt.hasSpecificFault(MtrCtrl.FRONT_RIGHT, FaultID.kOvercurrent);
      Robot.m_dt.hasSpecificFault(MtrCtrl.FRONT_RIGHT, FaultID.kStall);
      Robot.m_dt.hasSpecificFault(MtrCtrl.FRONT_RIGHT, FaultID.kMotorFault);
      Robot.m_dt.hasSpecificFault(MtrCtrl.FRONT_RIGHT, FaultID.kSensorFault);
      Robot.m_dt.hasSpecificFault(MtrCtrl.FRONT_RIGHT, FaultID.kStall);
    }
    if (Robot.m_dt.hasFault(MtrCtrl.BACK_LEFT)) {
      Robot.m_dt.hasSpecificFault(MtrCtrl.BACK_LEFT, FaultID.kBrownout);
      Robot.m_dt.hasSpecificFault(MtrCtrl.BACK_LEFT, FaultID.kOvercurrent);
      Robot.m_dt.hasSpecificFault(MtrCtrl.BACK_LEFT, FaultID.kStall);
      Robot.m_dt.hasSpecificFault(MtrCtrl.BACK_LEFT, FaultID.kMotorFault);
      Robot.m_dt.hasSpecificFault(MtrCtrl.BACK_LEFT, FaultID.kSensorFault);
      Robot.m_dt.hasSpecificFault(MtrCtrl.BACK_LEFT, FaultID.kStall);
    }
    if (Robot.m_dt.hasFault(MtrCtrl.BACK_RIGHT)) {
      Robot.m_dt.hasSpecificFault(MtrCtrl.BACK_RIGHT, FaultID.kBrownout);
      Robot.m_dt.hasSpecificFault(MtrCtrl.BACK_RIGHT, FaultID.kOvercurrent);
      Robot.m_dt.hasSpecificFault(MtrCtrl.BACK_RIGHT, FaultID.kStall);
      Robot.m_dt.hasSpecificFault(MtrCtrl.BACK_RIGHT, FaultID.kMotorFault);
      Robot.m_dt.hasSpecificFault(MtrCtrl.BACK_RIGHT, FaultID.kSensorFault);
      Robot.m_dt.hasSpecificFault(MtrCtrl.BACK_RIGHT, FaultID.kStall);
    }

    SmartDashboard.putNumber("Front Left Position", m_dt.getFLPosition());
    SmartDashboard.putNumber("Back Left Position", m_dt.getBLPosition());
    SmartDashboard.putNumber("Front Right Position", m_dt.getFRPosition());
    SmartDashboard.putNumber("Back Right Position", m_dt.getBRPosition());

    SmartDashboard.putNumber("Front Left Velocity", m_dt.getFLVelocity());
    SmartDashboard.putNumber("Back Left Velocity", m_dt.getBLVelocity());
    SmartDashboard.putNumber("Front Right Velocity", m_dt.getFRVelocity());
    SmartDashboard.putNumber("Back Right Velocity", m_dt.getBRVelocity());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = new ManualAuto();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (m_control.getSelected().equals(Controls.Arcade)) {
      Robot.m_oi.isTank = false;
    }
    
    if (m_control.getSelected().equals(Controls.Tank)) {
      Robot.m_oi.isTank = true;
    }
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    if (m_control.getSelected().equals(Controls.Arcade)) {
      Robot.m_oi.isTank = false;
    }
    
    if (m_control.getSelected().equals(Controls.Tank)) {
      Robot.m_oi.isTank = true;
    }
    SmartDashboard.putNumber("scale", TeleopDrive.scale);
    m_vs.displayTargetData();
    

    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    LiveWindow.run();
  }
}
