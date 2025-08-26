package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SwerveDriveServo;

public final class Drawing {
    ;

    private Drawing() {

    }


    public static void drawRobot(Canvas c, Pose2d t, Telemetry tel) {
        final double ROBOT_RADIUS = 9;
        c.setStrokeWidth(1);
        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);

        double xcenter = t.getX();
        double ycenter = t.getY();
        double theta = t.getHeading();

        double P1x = xcenter + ROBOT_RADIUS * Math.cos(theta);
        double P1y = ycenter + ROBOT_RADIUS * Math.sin(theta);
        double P2x = xcenter;
        double P2y = ycenter;

        c.strokeLine(P1x, P1y, P2x, P2y);
    }

    public static void drawRobotWithModules(Canvas c, SwerveDriveServo drive, Telemetry tel) {
        Pose2d t = drive.getPoseInchUnits(drive.getPose());
        double flAngle = Units.degreesToRadians(45);
        final double ROBOT_RADIUS = 9;
        final double MOD_RADIUS = 3;
        c.setStrokeWidth(1);
        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);

        double xcenter = t.getX();
        double ycenter = t.getY();
        double theta = t.getHeading();

        double P1x = xcenter + ROBOT_RADIUS * Math.cos(theta);
        double P1y = ycenter + ROBOT_RADIUS * Math.sin(theta);
        double P2x = xcenter;
        double P2y = ycenter;

        double flx1 = xcenter + ROBOT_RADIUS * Math.cos(flAngle);
        double fly1 = xcenter + ROBOT_RADIUS * Math.cos(flAngle);
      //  c.strokeText(String.valueOf(Math.sin(drive.getStates()[0].angle.getDegrees(),flx1,fly1)

        double flx2 = flx1 + MOD_RADIUS * Math.sin(drive.getStates()[0].angle.getRadians());
        double fly2 = flx1 + MOD_RADIUS * Math.cos(drive.getStates()[0].angle.getRadians());

        c.strokeLine(P1x, P1y, P2x, P2y);
        c.strokeLine(flx1,fly1,flx2,fly2);
    }


}
