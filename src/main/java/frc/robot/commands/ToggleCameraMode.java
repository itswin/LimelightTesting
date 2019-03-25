/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Add your docs here.
 */
public class ToggleCameraMode extends InstantCommand {
  /**
   * Add your docs here.
   */
  public ToggleCameraMode() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").getDouble(0) == 0) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    }
  }

}
