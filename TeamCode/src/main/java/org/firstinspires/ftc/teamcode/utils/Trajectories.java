package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

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

    public Trajectory crossLine, curve;


    public Trajectories() {


        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                        1,
                        2)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(SwerveDriveConstants.swerveKinematics);

        // straight line
        crossLine = TrajectoryGenerator.generateTrajectory(

                List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1.5, .3, new Rotation2d(0))),
                config);


        // start right of power port curve back for straight shot
        curve = TrajectoryGenerator
                .generateTrajectory(

                        // Start at the origin facing the +X direction
                        new Pose2d(0, -1, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(0, 0), new Translation2d(0, -1)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(0, 0, new Rotation2d(0)),
                        config);

        // zig zag
        // Start at the origin facing the +X direction
//        Pose2d.kZero,
//                // Pass through these two interior waypoints, making an 's' curve path
//                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
//                // End 3 meters straight ahead of where we started, facing forward
//                new Pose2d(3, 0, Rotation2d.kZero),
        curve = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(.9, .9), new Translation2d(1, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(1.8, 0, new Rotation2d(0)),
                // Pass config
                config);


    }

}