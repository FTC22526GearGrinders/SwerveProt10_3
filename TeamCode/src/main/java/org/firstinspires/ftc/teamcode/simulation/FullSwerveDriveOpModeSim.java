package org.firstinspires.ftc.teamcode.simulation;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.Drawing;


@TeleOp(name = "FullSwerveSim", group = "Sim")
@Config

public class FullSwerveDriveOpModeSim extends CommandOpMode {
    public static boolean RUNT;
    //  public Telemetry telemetry;
    SwerveDriveSim swerveDriveSim;
    double speedDivisor = 2;
    double m_prevTimeSeconds;
    TrajectoriesSim trajectories;
    Trajectory traj;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    public void initialize() {
        swerveDriveSim = new SwerveDriveSim(this);
        trajectories = new TrajectoriesSim(swerveDriveSim);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            run();
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.right_bumper) {
                double y = -gamepad1.left_stick_y / speedDivisor;
                double x = gamepad1.left_stick_x / speedDivisor;
                double rx = gamepad1.right_stick_x;
                swerveDriveSim.drive(y, -x, rx, true);
            }

            if (!currentGamepad1.right_bumper && previousGamepad1.right_bumper) {
                swerveDriveSim.drive(0., 0., 0, true);
            }

            if (gamepad1.left_bumper) {
                swerveDriveSim.resetYaw();
            }

            if (currentGamepad1.a && !previousGamepad1.a) {
                new RunTrajectoryCommandSim(swerveDriveSim, trajectories.crossLine, this).schedule();
            }
            if (currentGamepad1.b && !previousGamepad1.b) {
                new RunTrajectoryCommandSim(swerveDriveSim, trajectories.curve, this).schedule();
            }

            if (currentGamepad1.x && !previousGamepad1.x) {

            }

            if (currentGamepad1.y && !previousGamepad1.y) {
            }


            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Pose2d inchPose = swerveDriveSim.converMetersPoseToInches(swerveDriveSim.getPose());
            Drawing.drawRobot(packet.fieldOverlay(), inchPose, telemetry);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }
    }

}