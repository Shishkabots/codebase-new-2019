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

  public static WPI_TalonSRX leftTalon;
  public static WPI_TalonSRX rightTalon;

  public static DifferentialDrive m_drive;

  public static DoubleSolenoid ds;
  
  public static Hatch m_hatch; 
  public static Intake m_intake;

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
  public static GripPipeline pipe = new GripPipeline();

  public static double[] d;
  public static VisionHelper v;
  public static CvSink cv;
  
  @Override
  public void robotInit() {
    theCamera = CameraServer.getInstance().startAutomaticCapture();
		//theCamera.setVideoMode(theCamera.enumerateVideoModes()[101]);
    theCamera.setResolution(1280, 720);
    theCamera.setExposureManual(50);
    theCamera.setBrightness(50);
    //v = new VisionHelper();
    cv = CameraServer.getInstance().getVideo();
    CvSource out = CameraServer.getInstance().putVideo("boi", 1280, 720);
    
    if(cv == null){
      SmartDashboard.putNumber("cv is null", 1);
    }
    else{
      SmartDashboard.putNumber("cv is null", 0);
    }
    
    // visionThread = new VisionThread(theCamera, pipe, pipeline -> {
    //   count++;
    //   if (!pipeline.filterContoursOutput().isEmpty()) {
    //         synchronized (imgLock) {
    //           MatOfPoint m = pipeline.filterContoursOutput().get(0);
    //           //d = v.findCenter(m);

    //           MatOfPoint2f m2 = new MatOfPoint2f(m.toArray());
    //           r = Imgproc.minAreaRect(m2);
    //           m_centerX = r.center.x;
              
    //           //rect2 = Imgproc.boundingRect(m);
    //           //m_centerX2 = rect2.x + (rect2.width / 2);
              
    //           System.out.println("CAMERA VALUE" + m_centerX);
		//  	      }
    //   }
    //   else{
    //     synchronized(imgLock){

    //       r = null;
    //       //rect2 = null;
    //     }
    //   }
		// }); 
    // visionThread.start();
    

    side = new WPI_VictorSPX(3);

    leftTalon = new WPI_TalonSRX(6);
    leftVictor = new WPI_VictorSPX(2);
    //SpeedControllerGroup m_left = new SpeedControllerGroup(leftTalon, leftVictor);

    rightTalon = new WPI_TalonSRX(5);
    rightVictor = new WPI_VictorSPX(1);
    //SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
  
    leftTalon.setSafetyEnabled(false);
    rightTalon.setSafetyEnabled(false);
    leftVictor.setSafetyEnabled(false);
    rightVictor.setSafetyEnabled(false);
    side.setSafetyEnabled(false);
    m_drive = new DifferentialDrive(leftTalon, rightTalon);
    
    rightVictor.follow(rightTalon);
    leftVictor.follow(leftTalon);

    leftTalon.setInverted(false);
    rightTalon.setInverted(false);

    leftTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    //rightTalon.setSensorPhase(true);
    //leftTalon.setSensorPhase(true);

    leftTalon.setSelectedSensorPosition(0);
    rightTalon.setSelectedSensorPosition(0);

    leftVictor.setInverted(true);
    rightVictor.setInverted(true);

    rightTalon.config_kP(0,.01);
    rightTalon.config_kI(0,0.001);
    rightTalon.config_kD(0,0.001);

    leftTalon.config_kP(0,.01);
    leftTalon.config_kI(0,0.001);
    leftTalon.config_kD(0,0.001);
    //m_drive.setRightSideInverted(false);
    m_drivetrain = new DriveTrain();
    ds = new DoubleSolenoid(0, 1);
    //ds.set(DoubleSolenoid.Value.kForward);
    m_hatch = new Hatch();
    m_intake = new Intake();
    //m_intake.setState("On");

    //e1 = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    //e2 = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
    //e1.setDistancePerPulse(6 *3.14);
    //e2.setDistancePerPulse(6*3.14);
    
    gyro = new AHRS(SPI.Port.kMXP);

    
    //led = new Spark(1);
    //led.set(0.41);


    m_oi = new OI();
    
    SmartDashboard.putNumber("Start: ", 0);
    SmartDashboard.putNumber("End: ", 0);
    // VictorSPX side = new VictorSPX(2);
    // TalonSRX leftTalon = new TalonSRX(5);
    // VictorSPX leftVictor = new VictorSPX(3);
    // //SpeedControllerGroup m_left = new SpeedControllerGroup(leftTalon, leftVictor);
    // TalonSRX rightTalon = new TalonSRX(6);
    // VictorSPX rightVictor = new VictorSPX(4);
    //SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);

    /*m_chooser.setDefaultOption("Default Auto", new DriveTrainControl());
    SmartDashboard.putData("Auto mode", m_chooser);

    leftTalon.set(ControlMode.PercentOutput, .2);
    rightTalon.set(ControlMode.PercentOutput, .2);
    leftVictor.set(ControlMode.PercentOutput, .2);
    rightVictor.set(ControlMode.PercentOutput, .2);
    side.set(ControlMode.PercentOutput, .2);*/

    
    //side.set(ControlMode.PercentOutput, .2);
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
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //rightTalon.set(ControlMode.PercentOutput,1);
    /*RotatedRect re;
    double ree;
    double cent;
    double[] cenn;
    synchronized (imgLock) {
        re = r;
        ree = count;
        //cenn = d;
        //cent = m_centerX2;
    }
    //SmartDashboard.putNumber("other: ", cent);
    if(re != null){
      SmartDashboard.putNumber("Angle: ", re.angle);
      SmartDashboard.putNumber("Height: ", re.size.height);
      SmartDashboard.putNumber("Width: ", re.size.width);
      SmartDashboard.putNumber("x: ", re.center.x);
      SmartDashboard.putNumber("y: ", re.center.y);
      //SmartDashboard.putNumber("new x: ", d[0]);
      //SmartDashboard.putNumber("mew y: ", d[1]);
    }
    else{
      SmartDashboard.putNumber("Angle: ", 0);
      SmartDashboard.putNumber("Height: ", 0);
      SmartDashboard.putNumber("Width: ", 0);
      SmartDashboard.putNumber("x: ", 0);
      SmartDashboard.putNumber("y: ", 0);
      //SmartDashboard.putNumber("new x: ", 0);
      //SmartDashboard.putNumber("mew y: ", 0);
    }
    SmartDashboard.putNumber("Number: ", ree);*/
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
    new TeleOpCommands().start();
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
