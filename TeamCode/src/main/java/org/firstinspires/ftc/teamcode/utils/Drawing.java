package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SwerveDriveServo;

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

        if (showModules) {
            c.setStrokeWidth(2);
            c.strokeText("X+",68,0,"",0);
            c.strokeText("Y+",0,68,"",0);
            double moduleLength = 5;
            double moduleOriginX = 36;
            double moduleOriginY = 36;
            double flX = moduleOriginX + moduleLength * Math.cos(swerveDrive.modules[0].getState().angle.getRadians());
            double flY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[0].getState().angle.getRadians());

            double frX = moduleOriginX + moduleLength * Math.cos(swerveDrive.modules[1].getState().angle.getRadians());
            double frY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[1].getState().angle.getRadians());

            double rlX = moduleOriginX  + moduleLength * Math.cos(swerveDrive.modules[2].getState().angle.getRadians());
            double rlY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[2].getState().angle.getRadians());

            double rrX = moduleOriginX  + moduleLength * Math.cos(swerveDrive.modules[3].getState().angle.getRadians());
            double rrY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[3].getState().angle.getRadians());

            c.strokeLine(moduleOriginX, moduleOriginY, flX, flY);
            c.strokeLine(moduleOriginX , moduleOriginY, frX, frY);
            c.strokeLine(moduleOriginX, moduleOriginY, rlX, rlY);
            c.strokeLine(moduleOriginX, moduleOriginY, rrX, rrY);

        }


    }


}
