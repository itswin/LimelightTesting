/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class CenterHorizontallyOnVisionTarget extends Command {
  public CenterHorizontallyOnVisionTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Centering horizontally");
    Robot.m_driveTrain.driveState = DriveTrain.DriveState.kAuto;
    Robot.m_driveTrain.horizontalPIDController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // return false;
    // return Robot.m_driveTrain.horizontalPIDController.onTarget() || Robot.m_driveTrain.driveState != DriveTrain.DriveState.kAuto;
    return Robot.m_driveTrain.centeringPIDsOnTarget() || Robot.m_driveTrain.driveState != DriveTrain.DriveState.kAuto;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Finished centering horizontally");
    Robot.m_driveTrain.driveState = DriveTrain.DriveState.kManual;
    Robot.m_driveTrain.horizontalPIDController.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    System.out.println("Centering horizontally interrupted");
    Robot.m_driveTrain.driveState = DriveTrain.DriveState.kManual;
    Robot.m_driveTrain.horizontalPIDController.reset();
  }
}
