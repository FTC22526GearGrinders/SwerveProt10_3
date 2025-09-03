package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SwerveDriveServo;
import org.opencv.core.Point;

public final class DrawModules {
    public static double moduleOriginX = 48;
    public static double moduleOriginY = 48;


    public static double flX;

    public static double flY;

    public static double frX;

    public static double frY;

    public static double rlX;

    public static double rlY;

    public static double rrX;

    public static double rrY;


    private DrawModules() {

    }


    public static void drawModules(Canvas c, SwerveDriveServo swerveDrive, int view) {
        c.setStrokeWidth(1);
        c.strokeText("Act", 12, 70, "", 0);
        c.strokeText("Cmd", 60, 70, "", 0);
        c.strokeText("Orig", 108, 70, "", 0);

        double moduleLength = 10;
        double speedMult = 3;

        {

            moduleOriginX = 48;
            moduleOriginY = 48;
            moduleLength += swerveDrive.modules[0].getState().speedMetersPerSecond * speedMult;
            flX = moduleOriginX + moduleLength * Math.cos(swerveDrive.modules[0].getState().angle.getRadians());
            flY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[0].getState().angle.getRadians());

            frX = moduleOriginX + moduleLength * Math.cos(swerveDrive.modules[1].getState().angle.getRadians());
            frY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[1].getState().angle.getRadians());

            rlX = moduleOriginX + moduleLength * Math.cos(swerveDrive.modules[2].getState().angle.getRadians());
            rlY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[2].getState().angle.getRadians());

            rrX = moduleOriginX + moduleLength * Math.cos(swerveDrive.modules[3].getState().angle.getRadians());
            rrY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[3].getState().angle.getRadians());

            showModules(c);


            moduleOriginX = 48;
            moduleOriginY = 0;
            moduleLength += swerveDrive.modules[0].newState.speedMetersPerSecond * speedMult;
            flX = moduleOriginX + moduleLength * Math.cos(swerveDrive.modules[0].newState.angle.getRadians());
            flY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[0].newState.angle.getRadians());

            frX = moduleOriginX + moduleLength * Math.cos(swerveDrive.modules[1].newState.angle.getRadians());
            frY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[1].newState.angle.getRadians());

            rlX = moduleOriginX + moduleLength * Math.cos(swerveDrive.modules[2].newState.angle.getRadians());
            rlY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[2].newState.angle.getRadians());

            rrX = moduleOriginX + moduleLength * Math.cos(swerveDrive.modules[3].newState.angle.getRadians());
            rrY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[3].newState.angle.getRadians());

            showModules(c);

            moduleOriginX = 48;
            moduleOriginY = -48;
            moduleLength += swerveDrive.modules[0].origState.speedMetersPerSecond * speedMult;
            flX = moduleOriginX + moduleLength * Math.cos(swerveDrive.modules[0].origState.angle.getRadians());
            flY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[0].origState.angle.getRadians());

            frX = moduleOriginX + moduleLength * Math.cos(swerveDrive.modules[1].origState.angle.getRadians());
            frY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[1].origState.angle.getRadians());

            rlX = moduleOriginX + moduleLength * Math.cos(swerveDrive.modules[2].origState.angle.getRadians());
            rlY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[2].origState.angle.getRadians());

            rrX = moduleOriginX + moduleLength * Math.cos(swerveDrive.modules[3].origState.angle.getRadians());
            rrY = moduleOriginY + moduleLength * Math.sin(swerveDrive.modules[3].origState.angle.getRadians());

            showModules(c);
        }


    }


    public static void showModules(Canvas c) {

        c.setStroke("red");
        c.strokeLine(moduleOriginX, moduleOriginY, flX, flY);
        c.setStroke("green");
        c.strokeLine(moduleOriginX, moduleOriginY, frX, frY);
//        c.setStroke("blue");
//        c.strokeLine(moduleOriginX, moduleOriginY, rlX, rlY);
//        c.setStroke("yellow");
//        c.strokeLine(moduleOriginX, moduleOriginY, rrX, rrY);

    }
}


