/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import frc.robot.drivetrain.Drivetrain;

/**
 * Add your docs here.
 */
public class AutoVisionDriving extends AbstractVisionDriving {

    @Override
    public double getThrottle() {
        double distanceToTarget = -(Drivetrain.getDrivetrain().getFrontCamera().getVerticalDegreesToTarget() + 9.7);
        double speed = distanceToTarget * 0.4;
        if (speed > 3) {
            return 3;
        }
        return speed;
      }

    
}
