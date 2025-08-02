package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import java.util.ArrayList;

public class ExampleTrajectory {

    public Trajectory generateTrajectory() {

        // 2018 cross scale auto waypoints.
        Pose2d sideStart = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0),
                Rotation2d.fromDegrees(0));
        Pose2d crossScale = new Pose2d(Units.feetToMeters(6), Units.feetToMeters(0),
                Rotation2d.fromDegrees(0));

        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(3), Units.feetToMeters(0)));
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(6), Units.feetToMeters(0)));

        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(1), Units.feetToMeters(2));
        config.setReversed(false);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                sideStart,
                interiorWaypoints,
                crossScale,
                config);
        return trajectory;
    }
}