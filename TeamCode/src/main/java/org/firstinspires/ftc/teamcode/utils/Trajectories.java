package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import org.firstinspires.ftc.teamcode.drive.SwerveDrive;
import org.firstinspires.ftc.teamcode.drive.SwerveDriveConstants;

import java.util.List;


/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


/**
 * Add your docs here.
 */
public class Trajectories {

    public Trajectory crossLine, example;
    public Trajectory trenchStartOne, trenchStartTwo, trenchStartThree;
    public Trajectory controlPanelStartOne, controlPanelStartTwo;
    public Trajectory centerStart, leftStart, rightStart;
    public Trajectory leftStartCurve, rightStartCurve;
    private SwerveDrive m_drive;
    private static double bottomRectAngle = -Math.toRadians(67.5);
    private static double topRectAngle = -Math.toRadians(22.5);

    public Trajectories(SwerveDrive drive) {
        m_drive = drive;

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                        SwerveDriveConstants.maxSpeedMetersPerSec,
                        SwerveDriveConstants.maxAccelerationMPSPS)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(SwerveDriveConstants.swerveKinematics);

        // straight line
        crossLine = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(13, 0, new Rotation2d(0)), new Pose2d(12, 0, new Rotation2d(0))),
                config);

        // straight line opposite power port
        centerStart = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(13, -5.7, new Rotation2d(0)),
                new Pose2d(12, -5.7, new Rotation2d(0))), config);

        // straight line left of power port
        leftStart = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(13, -5.2, new Rotation2d(0)),
                new Pose2d(12, -5.2, new Rotation2d(0))), config);

        // straight line right of power port
        rightStart = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(13, -6.3, new Rotation2d(0)),
                new Pose2d(12, -6.3, new Rotation2d(0))), config);

        // start left of power port curve back for straight shot
        leftStartCurve = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(13, -5.2, new Rotation2d(0)),
                new Pose2d(12, -5.3, new Rotation2d(0))), config);

        // start right of power port curve back for straight shot
        rightStartCurve = TrajectoryGenerator
                .generateTrajectory(
                        List.of(new Pose2d(13, -6.3, new Rotation2d(0)),
                                new Pose2d(12, -5.3, new Rotation2d(0))),
                        config);

        // zig zag
       example= TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);

        trenchStartOne = TrajectoryGenerator.generateTrajectory(
                // List.of(new Pose2d(2, 0, new Rotation2d(0)), new Pose2d(4, 0, new
                // Rotation2d(0))),
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
                new Pose2d(4, 0.1, new Rotation2d(0)),
                // pass config
                config);

        trenchStartTwo = TrajectoryGenerator.generateTrajectory(

                // List.of(new Pose2d(6.5, 0, new Rotation2d(0)), new Pose2d(2, 0, new
                // Rotation2d(0))),
                new Pose2d(6.5, 0, new Rotation2d(0)),
                List.of(new Translation2d(4.5, 0), new Translation2d(3, 0)),
                new Pose2d(2.0, 0.1, new Rotation2d(0)),
                // pass config
                config);

        trenchStartThree = TrajectoryGenerator.generateTrajectory(
                // List.of(new Pose2d(4, 0, new Rotation2d(0)), new Pose2d(1, 0, new
                // Rotation2d(0))),
                new Pose2d(4, 0, new Rotation2d(0)),
                List.of(new Translation2d(3, 0), new Translation2d(2, 0)),
                new Pose2d(1, 0.1, new Rotation2d(0)),
                // pass config
                config);


    }

}