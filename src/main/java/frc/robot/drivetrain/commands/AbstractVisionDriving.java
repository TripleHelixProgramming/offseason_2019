/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import com.team2363.controller.PIDController;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;

import static frc.robot.drivetrain.Drivetrain.getDrivetrain;

public abstract class AbstractVisionDriving extends Command {

  private PIDController controller = new PIDController(0.05, 0, 0);
  private Notifier notifier = new Notifier(this::calculate);

  public AbstractVisionDriving() {
    // Use requires() here to declare subsystem dependencies
    requires(getDrivetrain());
  }

  public abstract double getThrottle();

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    notifier.startPeriodic(0.001);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double angleToTarget = getDrivetrain().getFrontCamera().getRotationalDegreesToTarget();
    controller.setReference(getDrivetrain().getHeading() + angleToTarget);
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

  private void calculate() {
    double output = controller.calculate(getDrivetrain().getHeading());
    getDrivetrain().setVelocityOutput(getThrottle() + output, getThrottle() - output);
  }
}
