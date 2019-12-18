/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import com.team319.trajectory.Path;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PathToTarget extends CommandGroup {

  private PathFollower pathFollower;
  private WaitForTarget waitForTarget;

  public PathToTarget(Path path) {
    pathFollower = new PathFollower(path);
    waitForTarget = new WaitForTarget();

    addParallel(pathFollower); // Start the path
    addSequential(waitForTarget); // Wait until we find an acceptable target
    addSequential(new AutoVisionDriving(3, 3)); // Stop the path and drive towards the target until we get to a predetermined range
    addSequential(new RampDown(1.1, 0.05)); // Decellerate to the target
  }

  @Override
  public boolean isFinished() {
    // Either we finish the command normally or the path finishes before we find an acceptable target
    return super.isFinished() || pathFollower.isCompleted();
  }
}
