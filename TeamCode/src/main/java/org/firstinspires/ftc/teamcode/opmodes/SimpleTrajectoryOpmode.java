package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.ExampleTrajectory;
import org.firstinspires.ftc.teamcode.drive.SwerveDrive;

@TeleOp(name = "SimpleTrajTest", group = "Test")
@Config
public class SimpleTrajectoryOpmode extends CommandOpMode {

    public Telemetry m_telemetry;
    SwerveDrive swerveDrive;
    ExampleTrajectory extratraj;
    Trajectory traj;

    public void initialize() {
        swerveDrive = new SwerveDrive(this);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        m_telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        extratraj = new ExampleTrajectory();
    }

    @Override
    public void runOpMode() {
        initialize();
        traj = extratraj.generateTrajectory();
        waitForStart();
        while (opModeIsActive()) {
            run();
            m_telemetry.update();
            swerveDrive.runTrajectory(traj);

        }
    }
}