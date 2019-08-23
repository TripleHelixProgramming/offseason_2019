/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivetrain;

import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {

    private String name;

    public Camera(String name) {
        this.name = name;
    }

    public void setToCameraMode() {
        getDefault().getTable(name).getEntry("pipeline").setNumber(0);
    }
    
    public void setDockingMode() {
        getDefault().getTable(name).getEntry("pipeline").setNumber(1);
        getDefault().getTable(name).getEntry("ledMode").setNumber(0);
        // NetworkTableInstance.getDefault().getTable(name).getEntry("pipeline").setNumber(1);
        getDefault().getTable(name).getEntry("stream").setNumber(0);
    }

    public void setMiddleDockingMode() {
        getDefault().getTable(name).getEntry("pipeline").setNumber(2);
        getDefault().getTable(name).getEntry("ledMode").setNumber(0);
        // NetworkTableInstance.getDefault().getTable(name).getEntry("pipeline").setNumber(2);
        getDefault().getTable(name).getEntry("stream").setNumber(0);
    }

    public boolean isTargetFound() {
        double v = getDefault().getTable(name).getEntry("tx").getDouble(0);
        return v != 0;
    }

    public double getRotationalDegreesToTarget() {
        return getDefault().getTable(name).getEntry("tx").getDouble(0);
    }

    public double getVerticalDegreesToTarget() {
        return getDefault().getTable(name).getEntry("ty").getDouble(0);
    }

    public double getAreaOfTarget() {
        return getDefault().getTable(name).getEntry("ta").getDouble(0);
    }

    public void getType() {
        // SmartDashboard.putString("Type", "" + getDefault().getTable(name).getEntry("tcornx").getNumberArray(new Number [4])[0]);
    }

    public double getTargetSkew() {
        return getVerticalDegreesToTarget() / (getAreaOfTarget() - Math.abs(getRotationalDegreesToTarget() * 0.01));
    }

    public static void main(String... args) {
        Camera camera = new Camera("x");
        System.out.println(camera.getTargetSkew());
    }

    private class Coordinate {
        private Double x, y;

        public Coordinate(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }


}
