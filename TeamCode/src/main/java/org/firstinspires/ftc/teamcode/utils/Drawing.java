package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public final class Drawing {;

    private Drawing() {

    }


    public static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);

        double xcenter = t.getX();
        double ycenter = t.getY();
        double theta = Units.degreesToRadians(t.getHeading());

        double P1x = xcenter + ROBOT_RADIUS * Math.cos(theta);
        double P1y = ycenter + ROBOT_RADIUS * Math.sin(theta);

//        double P2x = xcenter + ROBOT_RADIUS * Math.cos(theta+Math.PI);
//        double P2y = ycenter + ROBOT_RADIUS * Math.sin(theta+Math.PI);
        double P2x = xcenter;
        double P2y = ycenter;

        c.strokeLine(P1x, P1y, P2x, P2y);

    }

}
