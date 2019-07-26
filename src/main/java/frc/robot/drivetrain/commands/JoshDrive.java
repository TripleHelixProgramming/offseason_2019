/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain.commands;

import com.team2363.commands.NormalizedArcadeDrive;

import frc.robot.drivetrain.Drivetrain;
import frc.robot.oi.OI;

/**
 * Add your docs here.
 */
public class JoshDrive extends NormalizedArcadeDrive {

    private static double THROTTLE_SCALAR = 1;

    public JoshDrive() {
        super(Drivetrain.getDrivetrain());
    }

    @Override
    protected double getThrottle() {
        return OI.getOI().getThrottle() * THROTTLE_SCALAR;
    }

    @Override
    protected double getTurn() {
        if (Math.abs(OI.getOI().getTurn()) < 0.05) {
            return 0;
        }
        return OI.getOI().getTurn() * 0.5;
    }

    @Override
    protected void useOutputs(double left, double right) {
        Drivetrain.getDrivetrain().setPercentOutput(left, right);
    }
}
