/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;
  
  public static AHRS m_navX;
  static {
    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      m_navX = new AHRS(SerialPort.Port.kUSB); 
    } catch (RuntimeException ex ) {
      System.out.println("Error instantiating navX-MXP:  " + ex.getMessage());
    }
  }

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public static DriveTrain m_driveTrain = new DriveTrain();
  public static CenteringRotation m_cR = new CenteringRotation();
  public static CenteringHorizontal m_cH = new CenteringHorizontal();
  public static CenteringVertical m_cV = new CenteringVertical();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    // m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

    m_driveTrain.initPIDs();
    m_navX.reset();
    m_driveTrain.zeroAngle = 0;
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(320,240);
    camera.setExposureManual(5);
    camera.setFPS(20);
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
    double xDisplacement = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double contourArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    // Horizontal
    double error = -xDisplacement / Math.sqrt(contourArea);
    double sign = Math.signum(error);
    // System.out.println(sign * Math.sqrt(error * sign) - Robot.m_driveTrain.kHorizontalSetpoint);
    // System.out.println(sign * Math.sqrt(error * sign));

    // Vertical
    // System.out.println(contourArea);
    // System.out.println(m_driveTrain.verticalPIDController.onTarget() + "\t" +
    //       m_driveTrain.horizontalPIDController.onTarget());
    
    // System.out.println(m_navX.getYaw());
    NetworkTableInstance.getDefault().getTable("RobotData").getEntry("batteryVoltage").setNumber(RobotController.getBatteryVoltage());
    NetworkTableInstance.getDefault().getTable("RobotData").getEntry("robotTime").setNumber(Timer.getFPGATimestamp());
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
    
    // m_navX.reset();
    // m_driveTrain.zeroAngle = 0;
    m_driveTrain.rotationPIDController.setSetpoint(getComparedYaw());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    // Slow down rotation
    double rotationScalar = .5;

    double speed = -Robot.m_oi.driveController.getLeftYAxis();
    double strafe = Robot.m_oi.driveController.getLeftXAxis();
    double rotation = Robot.m_oi.driveController.getRightXAxis() * rotationScalar;
    
    // Square the values to make driving less sensitive
    // speed = speed * speed * Math.signum(speed);
    // strafe = strafe * strafe * Math.signum(strafe);
    // rotation = rotation*rotation * Math.signum(rotation);
    
    switch(m_driveTrain.driveState) {
      case kManual:
        // Sends joystick values to PIDDriveTrain
        m_driveTrain.setInputJoystickSpeeds(speed, strafe, rotation);
        break;
      case kAuto:
        // Speeds are set in the command currently controlling the drivetrain

        // If joystick input, stop auto
        if(speed != 0 || strafe != 0 || rotation != 0) {
          m_driveTrain.driveState = DriveTrain.DriveState.kManual;
        }
        break;
      case kAutoHorizontal:
        m_driveTrain.setInputJoystickSpeeds(speed, 0, rotation);
      default:
        break;
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public static double getComparedYaw() {
    return (m_navX.getYaw() - m_driveTrain.zeroAngle) % 180;
  }
}
