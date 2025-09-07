package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.RunTrajectoryCommandSwerve;
import org.firstinspires.ftc.teamcode.drive.ServoDriveCommand;
import org.firstinspires.ftc.teamcode.drive.SwerveDriveServo;
import org.firstinspires.ftc.teamcode.utils.DrawModules;
import org.firstinspires.ftc.teamcode.utils.Drawing;
import org.firstinspires.ftc.teamcode.utils.Trajectories;


@TeleOp(name = "SwerveServoOpMode", group = "Test")
@Config
public class SwerveDriveServoOpMode extends CommandOpMode {

    //  public Telemetry telemetry;
    SwerveDriveServo swerveDrive;
    double speedDivisor = 2;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Trajectories trajectories;
    Trajectory traj;

    public void initialize() {
        swerveDrive = new SwerveDriveServo(this);
        register(swerveDrive);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        swerveDrive.setModuleOpenloop(false);

        swerveDrive.setDefaultCommand
                (new ServoDriveCommand(swerveDrive, gamepad1, false, this));

        swerveDrive.resetPose(new Pose2d(-1, 1, new Rotation2d()));
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            run();

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);


            if (gamepad1.a && !previousGamepad1.a) {
                swerveDrive.resetYaw();
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                new RunTrajectoryCommandSwerve(swerveDrive, trajectories.crossLine, this).schedule();
            }

            TelemetryPacket packet = new TelemetryPacket(false);
            packet.fieldOverlay()
                    .drawGrid(0, 0, 144, 144, 7, 7);
            packet.fieldOverlay().setStroke("#3F51B5");
            DrawModules.drawModules(packet.fieldOverlay(), swerveDrive, 0);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

//          TelemetryPacket packet = new TelemetryPacket();
//            packet.fieldOverlay().setStroke("#3F51B5");
//            Pose2d inchPose = swerveDrive.getPoseInchUnits(swerveDrive.getPose());
//            Drawing.drawRobot(packet.fieldOverlay(), swerveDrive, true, telemetry);
 //           FtcDashboard.getInstance().sendTelemetryPacket(packet);

            double wheelAngle = swerveDrive.modules[0].getWheelAngleDeg();
            telemetry.addData("A0PotVolts", "%5.3f",swerveDrive.modules[0].getPotVolts());
          //  telemetry.addData("A0FromPot", "%5.3f",wheelAngle);
          //  telemetry.addData("Speed0", "%5.3f",swerveDrive.modules[0].newState.speedMetersPerSecond);
            telemetry.addData("Tgt", "%5.3f",swerveDrive.modules[0].newState.angle.getDegrees());
            telemetry.addData("TgtWrap", "%5.3f",swerveDrive.modules[0].targetAngleWrapped);
            telemetry.addData("VEPID", "%5.3f",swerveDrive.modules[0].voltsErrorPID);

            telemetry.addData("A0SVPos", "%5.3f",swerveDrive.modules[0].getServoPosition());
            telemetry.addData("Yaw", "%5.3f",swerveDrive.getHeading().getDegrees());

//
//            telemetry.addData("A1FromPot", swerveDrive.modules[1].getWheelAngleDeg());
//            telemetry.addData("Speed1", swerveDrive.modules[1].newState.speedMetersPerSecond);
//            telemetry.addData("A1Tgt", swerveDrive.modules[1].newState.angle.getDegrees());
//            telemetry.addData("A1SVPos", swerveDrive.modules[1].getServoPosition());
//            telemetry.addData("A1DegFromServo", swerveDrive.modules[1].getDegreesFromServoPosition());

//            telemetry.addData("A2FromPot", swerveDrive.modules[2].getWheelAngleDeg());
//            telemetry.addData("Speed2", swerveDrive.modules[2].newState.speedMetersPerSecond);
//            telemetry.addData("A2Tgt", swerveDrive.modules[2].newState.angle.getDegrees());
//            telemetry.addData("A2SVPos", swerveDrive.modules[2].getServoPosition());
//            telemetry.addData("A2DegFromServo", swerveDrive.modules[2].getDegreesFromServoPosition());
//
//            telemetry.addData("A3FromPot", swerveDrive.modules[3].getWheelAngleDeg());
//            telemetry.addData("Speed3", swerveDrive.modules[3].newState.speedMetersPerSecond);
//            telemetry.addData("A3Tgt", swerveDrive.modules[3].newState.angle.getDegrees());
//            telemetry.addData("A3SVPos", swerveDrive.modules[3].getServoPosition());
//            telemetry.addData("A3DegFromServo", swerveDrive.modules[3].getDegreesFromServoPosition());

            telemetry.update();
        }
    }

}