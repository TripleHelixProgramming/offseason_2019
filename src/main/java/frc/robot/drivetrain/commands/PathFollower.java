/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import com.team2363.commands.HelixFollower;
import com.team2363.controller.PIDController;
import com.team319.trajectory.Path;

import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.Drivetrain.CommandType;
import frc.robot.drivetrain.Drivetrain.ControlType;

public class PathFollower extends HelixFollower {

    private Drivetrain drivetrain = Drivetrain.getDrivetrain();

    private PIDController headingController = new PIDController(15, 0, 0, 0.001);
    private PIDController distanceController = new PIDController(10, 0, 0, 0.001);

    public PathFollower(Path path) {
        super(path);
        requires(drivetrain);
    }

    @Override
    public void resetDistance() {
        drivetrain.resetEncoders();
    }

    @Override
    public PIDController getHeadingController() {
        return headingController;
    }

    @Override
    public PIDController getDistanceController() {
        return distanceController;
    }

    @Override
    public double getCurrentDistance() {
        return (drivetrain.getLeftPosition() + drivetrain.getRightPosition()) / 2.0;
    }

    @Override
    public double getCurrentHeading() {
        return Math.toRadians(drivetrain.getHeading());
    }

    @Override
    public void useOutputs(double left, double right) {
        drivetrain.setSetpoint(CommandType.FPS, ControlType.VELOCITY, left, right);
	}
}
