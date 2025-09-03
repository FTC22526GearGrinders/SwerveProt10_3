package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SwerveDriveServo;
import org.opencv.core.Point;

public final class Drawing {
    ;

    private Drawing() {

    }


    public static void drawRobot(Canvas c, SwerveDriveServo swerveDrive, boolean showModules, Telemetry tel) {
        Pose2d t = swerveDrive.getPoseInchUnits(swerveDrive.getPose());
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


}

