/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import frc.robot.drivetrain.Drivetrain;
import frc.robot.oi.OI;

public class ManualVisionDriving extends AbstractVisionDriving {

  @Override
  public double getThrottle() {
    return OI.getOI().getThrottle() * Drivetrain.MAX_VELOCITY_IN_FPS;
  }
}
