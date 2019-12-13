/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import com.team2363.logger.HelixEvents;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.drivetrain.Drivetrain;

public class WaitForTarget extends Command {

  @Override
  protected boolean isFinished() {
    return Drivetrain.getDrivetrain().getFrontCamera().isTargetFound();
  }

  @Override
  protected void end() {
    HelixEvents.getInstance().addEvent("Camera", "Found target");
  }
}
