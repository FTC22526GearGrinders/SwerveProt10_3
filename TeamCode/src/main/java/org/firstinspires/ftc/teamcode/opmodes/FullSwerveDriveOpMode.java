package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SwerveDrive;
import org.firstinspires.ftc.teamcode.utils.Drawing;


@TeleOp(name = "FullSwerveTest", group = "Test")
@Config
public class FullSwerveDriveOpMode extends CommandOpMode {

    //  public Telemetry telemetry;
    SwerveDrive swerveDrive;
    double speedDivisor = 2;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    public void initialize() {
        swerveDrive = new SwerveDrive(this);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        swerveDrive.setModuleOpenloop(false);
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
                swerveDrive.drive(y, -x, rx, true);
            }

            if (!currentGamepad1.right_bumper && previousGamepad1.right_bumper) {
                swerveDrive.drive(0., 0., 0, true);
            }


        if (gamepad1.a) {
            swerveDrive.resetYaw();
        }
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Pose2d inchPose = swerveDrive.getPoseInchUnits(swerveDrive.getPose());
        Drawing.drawRobot(packet.fieldOverlay(), inchPose, telemetry);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();
    }
}
}