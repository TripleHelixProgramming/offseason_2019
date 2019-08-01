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

    public double getTargetSkew() {
        int[] xFromLimelight = {2, 6, 1, 7};
        int[] yFromLimelight = {3, 1, 4, 8};
        
        List<Coordinate> coordinates = new ArrayList<>();

        for (int i = 0; i < xFromLimelight.length; i++) {
            coordinates.add(new Coordinate(xFromLimelight[i], yFromLimelight[i]));
        }

        coordinates = coordinates.stream().sorted((c1, c2) -> c1.y.compareTo(c2.y)).limit(4).collect(Collectors.toList());
        Coordinate left = coordinates.stream().sorted((c1, c2) -> c1.x.compareTo(c2.x)).limit(1).collect(Collectors.toList()).get(0);
        Coordinate right = coordinates.stream().sorted((c1, c2) -> c2.x.compareTo(c1.x)).limit(1).collect(Collectors.toList()).get(0);
    
        return (double)(right.y - left.y) / (double)(right.x - left.x);
    }

    public static void main(String... args) {
        Camera camera = new Camera("x");
        System.out.println(camera.getTargetSkew());
    }

    private class Coordinate {
        private Integer x, y;

        public Coordinate(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }
}
