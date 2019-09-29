/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.GripPipeline;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Spark;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // 1. subsystem declarations + initializations (static)
  public static DriveTrain m_drivetrain;
  //public static DT m_DT = new DT();
  public static OI m_oi;

  // 2. declaration of sendablechooser (send autonomous ops to driverstation) and autonomous options
  //Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   * 
   */
  public static WPI_VictorSPX side;
  public static WPI_VictorSPX leftVictor;
  public static WPI_VictorSPX rightVictor;
  public static WPI_VictorSPX intake;

  public static WPI_TalonSRX leftTalon;
  public static WPI_TalonSRX rightTalon;

  public static WPI_VictorSPX victortemp;
  public static DifferentialDrive m_drive;

  public static DoubleSolenoid ds;
  
  public static Hatch m_hatch; 
  public static CargoIntake m_intake;

  private VisionThread visionThread;
  public static final Object imgLock = new Object();
  public static double m_centerX = 0.0;
  public static RotatedRect r;
  public static Rect rect2;
  public static double m_centerX2;

  public static Encoder e1;
  public static Encoder e2;

  public static AHRS gyro;
  public static int count;

  public static Spark led;

  public static UsbCamera theCamera;
  public static UsbCamera theCamera2;
  public static GripPipeline pipe = new GripPipeline();

  public static double[] d;
  public static VisionHelper v;
  public static CvSink cv;
  public static CvSource out;
  public static boolean testing;

  Mat input = new Mat();
  public static Thread t;
  
  @Override
  public void robotInit() {
    //toggle testing mode
    testing = true;
    GripPipeline wede = new GripPipeline();
    t = new Thread(() -> {
      //Camera Set up
      theCamera = CameraServer.getInstance().startAutomaticCapture(0);
      theCamera2 = CameraServer.getInstance().startAutomaticCapture(1);
      //theCamera.setVideoMode(theCamera.enumerateVideoModes()[101]);
      theCamera.setFPS(7);
      theCamera.setResolution(320, 240);
      //theCamera.setExposureManual(50);
      theCamera.setBrightness(30);
      theCamera.setExposureAuto();
      theCamera.setWhiteBalanceAuto();

      theCamera2.setFPS(7);
      theCamera2.setResolution(320, 240);
      //theCamera.setExposureManual(50);
      theCamera2.setBrightness(30);
      theCamera2.setExposureAuto();
      theCamera2.setWhiteBalanceAuto();

      cv = CameraServer.getInstance().getVideo();
      out = CameraServer.getInstance().putVideo("Prefilter", 320, 240);

      Mat input = new Mat();
      //Check for vision availability
      while(!Thread.interrupted()) {
        cv.grabFrame(input);
        wede.process(input);
        if(wede.filterContoursOutput().size() == 1) {
          SmartDashboard.putString("Can I run vision? ", "YES");
        }else {
          SmartDashboard.putString("Can I run vision? ", "NO");
        }
      }
      
    });
    t.start();
    
    

    side = new WPI_VictorSPX(2);
    intake = new WPI_VictorSPX(4);
    leftTalon = new WPI_TalonSRX(6);
    leftVictor = new WPI_VictorSPX(3);
    rightTalon = new WPI_TalonSRX(5);
    rightVictor = new WPI_VictorSPX(1);
    victortemp = new WPI_VictorSPX(0);
  //Disable Safety
    leftTalon.setSafetyEnabled(false);
    rightTalon.setSafetyEnabled(false);
    leftVictor.setSafetyEnabled(false);
    rightVictor.setSafetyEnabled(false);
    side.setSafetyEnabled(false);
    victortemp.setSafetyEnabled(false);
    intake.setSafetyEnabled(false);
    //Victor shadows Talon
    rightVictor.follow(rightTalon);
    leftVictor.follow(leftTalon);

    leftTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightTalon.setSensorPhase(false);
    leftTalon.setSensorPhase(true);

    leftTalon.setSelectedSensorPosition(0);
    rightTalon.setSelectedSensorPosition(0);

    // change these as necessary dependign on drive
    leftTalon.setInverted(false);
    rightTalon.setInverted(false);
    leftVictor.setInverted(false);
    rightVictor.setInverted(false);
    //victortemp.setInverted(false); do we need this

    rightTalon.setNeutralMode(NeutralMode.Coast);
    leftTalon.setNeutralMode(NeutralMode.Coast);
    rightVictor.setNeutralMode(NeutralMode.Coast);
    leftVictor.setNeutralMode(NeutralMode.Coast);

    // in ticks per 100 ms, or in terms of voltage?
    // double maxSpeedTurn = 8;
    // double maxSpeedDrive = 8;
    // rightTalon.configClosedLoopPeakOutput(1, maxSpeedTurn);
    // leftTalon.configClosedLoopPeakOutput(1, maxSpeedTurn);
    // rightTalon.configClosedLoopPeakOutput(0, maxSpeedDrive);
    // leftTalon.configClosedLoopPeakOutput(0, maxSpeedDrive);

    // 0.1 kf is about 22 rotations/second, which is really really really fast (max speed)
    // I guess 0.005 giving 1 rotation/second is cool for indoors testing and the real will want to
    // be a little faster than that
    rightTalon.config_kF(0, 0.3);
    rightTalon.config_kP(0, 0.0); //3.0
    rightTalon.config_kD(0, 0); //80
    rightTalon.config_kI(0, 0.01); //0.01

    leftTalon.config_kF(0, 0.3);
    leftTalon.config_kP(0, 0.0); //3.0
    leftTalon.config_kD(0, 0); //80
    leftTalon.config_kI(0, 0.0); //0.01

    m_drive = new DifferentialDrive(leftTalon, rightTalon);
    m_drivetrain = new DriveTrain();
    //Set up solenoid
    ds = new DoubleSolenoid(0, 1);
    ds.set(DoubleSolenoid.Value.kForward);
    m_hatch = new Hatch();
    m_intake = new CargoIntake();
    //m_intake.setState("On");

    gyro = new AHRS(SPI.Port.kMXP);    
    led = new Spark(0);
    led.set(0.45);
    // go blue or red depending on ds input
    m_oi = new OI();


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
    //ds.set(Value.kForward);
    /*m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     * 
     */

    // schedule the autonomous command (example)
    /*if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }*/
    //new VisionProcess().start();
    //SmartDashboard.putNumber("heerer", value)
    new TeleOpCommands().start();
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
    /*if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }*/
    if(testing) {
      //SmartDashboard.putString("Progress:", "Reached T_OP Init");
    }
    new TeleOpCommands().start();
    
    //new VisionProcess().start();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    
  }

  @Override
  public void testInit(){
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
