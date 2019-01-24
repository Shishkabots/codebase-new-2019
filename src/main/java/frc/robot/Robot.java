/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // 1. subsystem declarations + initializations (static)
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static DT m_DT = new DT();
  public static OI m_oi;

  // 2. declaration of sendablechooser (send autonomous ops to driverstation) and autonomous options
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // WPI_VictorSPX side = new WPI_VictorSPX(2);

    // WPI_TalonSRX leftTalon = new WPI_TalonSRX(5);
    // WPI_VictorSPX leftVictor = new WPI_VictorSPX(3);
    // //SpeedControllerGroup m_left = new SpeedControllerGroup(leftTalon, leftVictor);

    // WPI_TalonSRX rightTalon = new WPI_TalonSRX(6);
    // WPI_VictorSPX rightVictor = new WPI_VictorSPX(4);
    // //SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
  
    // //DifferentialDrive m_drive = new DifferentialDrive(leftTalon, rightTalon);

    VictorSPX side = new VictorSPX(2);

    TalonSRX leftTalon = new TalonSRX(5);
    VictorSPX leftVictor = new VictorSPX(3);
    //SpeedControllerGroup m_left = new SpeedControllerGroup(leftTalon, leftVictor);

    TalonSRX rightTalon = new TalonSRX(6);
    VictorSPX rightVictor = new VictorSPX(4);
    //SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);

    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

    leftTalon.set(ControlMode.PercentOutput, 50);
    rightTalon.set(ControlMode.PercentOutput, 50);

    leftVictor.set(ControlMode.PercentOutput, 50);
    rightVictor.set(ControlMode.PercentOutput, 50);
    side.set(ControlMode.PercentOutput, 50);
  }

  @Override
  public void robotPeriodic() {
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
    m_autonomousCommand = m_chooser.getSelected();

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
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
