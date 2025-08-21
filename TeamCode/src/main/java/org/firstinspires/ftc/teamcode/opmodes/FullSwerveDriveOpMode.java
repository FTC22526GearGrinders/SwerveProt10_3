package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.RunTrajectoryCommand;
import org.firstinspires.ftc.teamcode.drive.SwerveDrive;
import org.firstinspires.ftc.teamcode.utils.Drawing;
import org.firstinspires.ftc.teamcode.utils.Trajectories;


@TeleOp(name = "FullSwerveTest", group = "Test")
@Config
public class FullSwerveDriveOpMode extends CommandOpMode {

    //  public Telemetry telemetry;
    SwerveDrive swerveDrive;
    double speedDivisor = 2;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Trajectories trajectories;
    Trajectory traj;

    public void initialize() {
        swerveDrive = new SwerveDrive(this);
        register(swerveDrive);
        trajectories = new Trajectories(swerveDrive);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        swerveDrive.setModuleOpenloop(false);
        swerveDrive.setDefaultCommand
                (new DriveCommand(swerveDrive,
                        gamepad1,
                        true,
                        this));

    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            run();

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (!currentGamepad1.right_bumper && previousGamepad1.right_bumper) {
                swerveDrive.drive(0., 0., 0, true);
            }


            if (gamepad1.a) {
                swerveDrive.resetYaw();
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                new RunTrajectoryCommand(swerveDrive, trajectories.crossLine, this).schedule();
            }


            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Pose2d inchPose = swerveDrive.getPoseInchUnits(swerveDrive.getPose());
            Drawing.drawRobot(packet.fieldOverlay(), inchPose, telemetry);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.addData("Speed", swerveDrive.getStates()[0].speedMetersPerSecond);
            telemetry.addData("Angle0", swerveDrive.modules[0].getWheelAngleDeg());


            telemetry.addData("Angle1", swerveDrive.modules[1].getWheelAngleDeg());


            telemetry.addData("Angle2", swerveDrive.modules[2].getWheelAngleDeg());


            telemetry.addData("Angle3", swerveDrive.modules[3].getWheelAngleDeg());


            telemetry.update();
        }
    }
}