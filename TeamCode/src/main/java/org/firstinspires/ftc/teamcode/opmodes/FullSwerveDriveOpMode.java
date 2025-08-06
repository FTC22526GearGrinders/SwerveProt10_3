package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SwerveDrive;
import org.firstinspires.ftc.teamcode.drive.SwerveDriveAutoLogged;
import org.firstinspires.ftc.teamcode.utils.Drawing;

import Ori.Coval.Logging.AutoLogManager;
import Ori.Coval.Logging.Logger.KoalaLog;

@TeleOp(name = "FullSwerveTest", group = "Test")
@Config
public class FullSwerveDriveOpMode extends CommandOpMode {

    public Telemetry telemetry;
    SwerveDrive swerveDrive;
    double speedDivisor =2;

    public void initialize() {
        swerveDrive = new SwerveDriveAutoLogged(this);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        swerveDrive.setModuleOpenloop(false);
        KoalaLog.setup(hardwareMap);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            run();
            AutoLogManager.periodic();
            if (gamepad1.right_bumper) {
                double y = -gamepad1.left_stick_y / speedDivisor;
                double x = gamepad1.left_stick_x / speedDivisor;
                double rx = gamepad1.right_stick_x ;
                swerveDrive.drive(y, -x, rx, true);
            }
            if (gamepad1.a) {
                swerveDrive.resetYaw();
            }
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), swerveDrive.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}