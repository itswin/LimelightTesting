/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.PIDs;

import edu.wpi.first.wpilibj.PIDOutput;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class DriveTrainStrafePIDOutput implements PIDOutput {

    @Override
    public void pidWrite(double output) {
        // Robot.m_driveTrain.setInputAutoStrafeSpeed(output);
    }
}
