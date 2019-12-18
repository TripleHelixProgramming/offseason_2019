/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.Drivetrain.CommandType;
import frc.robot.drivetrain.Drivetrain.ControlType;

public class VelocityTuning extends Command {
  public VelocityTuning() {

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Drivetrain.getDrivetrain());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Drivetrain.getDrivetrain().setSetpoint(CommandType.FPS, ControlType.VELOCITY, -2, 2);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
