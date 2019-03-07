/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import frc.robot.Robot;
import frc.robot.PIDs.*;
import frc.robot.commands.DriveTrainBaseCommand;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  private VictorSP frontLeft;
  private VictorSP frontRight;
  private VictorSP backLeft;
  private VictorSP backRight;

  private MecanumDrive m_drive;

  // PIDs
  private final double kRotationP = .025;
  private final double kRotationI = 0;
  private final double kRotationD = 0;
  private final double kRotationF = 0;
  private final double kRotationPeriod = .01;
  private final double kRotationAbsoluteTolerance = 2;
  public PIDController rotationPIDController;
  public DriveTrainRotationPIDOutput rotationPIDOutput;

  private final double kHorizontalP = 0;
  private final double kHorizontalI = 0;
  private final double kHorizontalD = 0;
  private final double kHorizontalF = 0;
  private final double kHorizontalPeriod = 0.01;
  private final double kHorizontalAbsoluteTolerance = 10; // TODO: Change
  public PIDController horizontalPIDController;
  public HorizontalDistancePIDSource horizontalDistancePIDSource;
  public DriveTrainStrafePIDOutput strafePIDOutput;

  private final double kVerticalP = 0;
  private final double kVerticalI = 0;
  private final double kVerticalD = 0;
  private final double kVerticalF = 0;
  private final double kVerticalPeriod = 0.01;
  private final double kVerticalAbsoluteTolerance = 10; // TODO: Change
  public PIDController verticalPIDController;
  public VerticalDistancePIDSource verticalDistancePIDSource;
  public DriveTrainStrafePIDOutput speedPIDOutput;
  
  // Joystick speeds updated in periodic
  private double inputJoystickSpeed = 0;
  private double inputJoystickStrafeSpeed = 0;
  private double inputJoystickRotationSpeed = 0;

  // Auto speeds
  private double inputAutoSpeed = 0;
  private double inputAutoStrafeSpeed = 0;
  private double inputAutoRotationSpeed = 0;

  // Current speed applied to motors
  private double currentSpeed = 0;
  private double currentStrafeSpeed = 0;
  private double currentRotationSpeed = 0;

  public DriveState driveState = DriveState.kManual;
  public enum DriveState {
    kAuto, kManual
  }

  public DriveTrain() {
    frontLeft = new VictorSP(0);
    frontRight = new VictorSP(1);
    backLeft = new VictorSP(2);
    backRight = new VictorSP(3);

    m_drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

    rotationPIDOutput = new DriveTrainRotationPIDOutput();
    rotationPIDController = new PIDController(kRotationP, kRotationI, kRotationD, kRotationF, Robot.m_navX, rotationPIDOutput, kRotationPeriod);
    rotationPIDController.setOutputRange(-.25, .25);
    rotationPIDController.setInputRange(-180, 180);
    rotationPIDController.setAbsoluteTolerance(kRotationAbsoluteTolerance);
    rotationPIDController.enable();

    horizontalDistancePIDSource = new HorizontalDistancePIDSource();
    horizontalPIDController = new PIDController(kHorizontalP, kHorizontalI, kHorizontalD, kHorizontalF, horizontalDistancePIDSource, strafePIDOutput, kHorizontalPeriod);
    horizontalPIDController.setOutputRange(-1, 1);
    horizontalPIDController.setInputRange(-100, 100); // TODO: Change
    horizontalPIDController.setAbsoluteTolerance(kHorizontalAbsoluteTolerance);
    horizontalPIDController.disable();

    verticalDistancePIDSource = new VerticalDistancePIDSource();
    verticalPIDController = new PIDController(kVerticalP, kVerticalI, kVerticalD, kVerticalF, verticalDistancePIDSource, speedPIDOutput, kVerticalPeriod);
    verticalPIDController.setOutputRange(-.5, .5);
    verticalPIDController.setInputRange(-100, 100); // TODO: Change
    verticalPIDController.setAbsoluteTolerance(kVerticalAbsoluteTolerance);
    verticalPIDController.disable();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveTrainBaseCommand());
  }

  public void drive(double speed, double strafe, double rotation) {
    m_drive.driveCartesian(speed, strafe, rotation);
  }

  public void stop() {
    rotationPIDController.setSetpoint(Robot.m_navX.getYaw());

    drive(0, 0, 0);
  }

  
  // ********** Methods so that updating inputs work from periodic ********** //
  public void setInputJoystickSpeeds(double speed, double strafe, double rotation) {
    inputJoystickSpeed = speed;
    inputJoystickStrafeSpeed = strafe;
    inputJoystickRotationSpeed = rotation;
  }

  public double getInputJoystickSpeed() {
    return inputJoystickSpeed;
  }

  public double getInputJoystickStrafeSpeed() {
    return inputJoystickStrafeSpeed;
  }

  public double getInputJoystickRotationSpeed() {
    return inputJoystickRotationSpeed;
  }
  
  // ********** Auto Input speed methods ********** //
  public void setInputAutoSpeeds(double speed, double strafe, double rotation) {
    inputAutoSpeed = speed;
    inputAutoStrafeSpeed = strafe;
    inputAutoRotationSpeed = rotation;
  }

  public double getInputAutoSpeed() {
    return inputAutoSpeed;
  }

  public double getInputAutoStrafeSpeed() {
    return inputAutoStrafeSpeed;
  }

  public double getInputAutoRotationSpeed() {
    return inputAutoRotationSpeed;
  }

  public void setInputAutoSpeed(double speed) {
    inputAutoSpeed = speed;
  }

  public void setInputAutoStrafeSpeed(double strafe) {
    inputAutoStrafeSpeed = strafe;
  }

  public void setInputAutoRotationSpeed(double rotation) {
    inputAutoRotationSpeed = rotation;
  }

  // ********** Current speed methods ********** //
  public void setCurrentSpeeds(double speed, double strafe, double rotation) {
    currentSpeed = speed;
    currentStrafeSpeed = strafe;
    currentRotationSpeed = rotation;
  }

  public void setCurrentSpeed(double speed) {
    currentSpeed = speed;
  }

  public void setCurrentStrafeSpeed(double strafe) {
    currentStrafeSpeed = strafe;
  }

  public void setCurrentRotationSpeed(double rotation) {
    currentRotationSpeed = rotation;
  }

  public double getCurrentSpeed() {
    return currentSpeed;
  }

  public double getCurrentStrafeSpeed() {
    return currentStrafeSpeed;
  }

  public double getCurrentRotationSpeed() {
    return currentRotationSpeed;
  }

  public void enableCenteringPIDs() {
    verticalPIDController.enable();
    horizontalPIDController.enable();
  }

  public void disableCenteringPIDs() {
    verticalPIDController.disable();
    horizontalPIDController.disable();
  }
}
