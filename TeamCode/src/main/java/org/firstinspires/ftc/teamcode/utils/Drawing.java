package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        tel.addData("DRXC",xcenter);

        double P1x = xcenter + ROBOT_RADIUS * Math.cos(theta);
        double P1y = ycenter + ROBOT_RADIUS * Math.sin(theta);
//
//        tel.addData("P1X",P1x);
//        tel.addData("DRTH",theta);

        double P2x = xcenter;
        double P2y = ycenter;

        c.strokeLine(P1x, P1y, P2x, P2y);

    }

}
