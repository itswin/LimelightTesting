/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class StaggeredCenterOnVisionTarget extends CommandGroup {
  /**
   * Add your docs here.
   */
  public StaggeredCenterOnVisionTarget() {
    addParallel(new CenterRotationOnVisionTarget());
    addParallel(new DelayedCenterHorizontallyOnVisionTarget());
    addSequential(new DelayedCenterVerticallyOnVisionTarget(1.5));
  }
}
