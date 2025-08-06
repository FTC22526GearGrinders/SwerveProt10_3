/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SwerveDrive;
import org.firstinspires.ftc.teamcode.drive.SwerveDriveConstants;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@TeleOp(name = "SwerveDriveTune", group = "Tune")
//@Disabled
@Config
public class SwerveDriveTune extends CommandOpMode {


    public static double[] DRIVEKP = {.01, .01, .01, .01};
    public static double[] DRIVEKI = {.0, .0, .0, .0};
    public static double[] DRIVEKD = {.0, .0, .0, .0};

    public static double[] DRIVEKS = {.01, .01, .01, .01};
    public static double[] DRIVEKV = {SwerveDriveConstants.calcKV, SwerveDriveConstants.calcKV, SwerveDriveConstants.calcKV, SwerveDriveConstants.calcKV};
    public static double[] DRIVEKA = {.0, .0, .0, .0};

    public static boolean ALL_SAME = true;

    SwerveDrive swerveDrive;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void initialize() {

    }

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        // Wait for the game to start (driver presses START)
        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                if (ALL_SAME) {
                    for (int i = 1; i < swerveDrive.modules.length; i++) {
                        DRIVEKP[i] = DRIVEKP[0];
                        DRIVEKI[i] = DRIVEKI[0];
                        DRIVEKD[i] = DRIVEKD[0];
                    }
                }
                for (int i = 0; i < swerveDrive.modules.length; i++) {
                    swerveDrive.setModuleDriveKP(i, DRIVEKP[i]);
                    swerveDrive.setModuleDriveKI(i, DRIVEKI[i]);
                    swerveDrive.setModuleDriveKD(i, DRIVEKD[i]);
                }
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                if (ALL_SAME) {
                    for (int i = 1; i < swerveDrive.modules.length; i++) {
                        DRIVEKS[i] = DRIVEKS[0];
                        DRIVEKV[i] = DRIVEKV[0];
                        DRIVEKA[i] = DRIVEKA[0];
                    }
                }
                for (int i = 0; i < swerveDrive.modules.length; i++) {
                    swerveDrive.modules[i].createFeedForward(DRIVEKS[i], DRIVEKV[i], DRIVEKA[i]);
                }
            }
            double driveSpeed = gamepad1.left_stick_y * SwerveDriveConstants.maxSpeedMetersPerSec;

            if (currentGamepad1.dpad_up) {
                swerveDrive.setModuleOpenloop(true);
            }

            if (currentGamepad1.dpad_down) {
                swerveDrive.setModuleOpenloop(false);
            }


            if (currentGamepad1.a && !previousGamepad1.a) {
                swerveDrive.setModuleStates(new SwerveModuleState(driveSpeed, new Rotation2d(0)));
            }
            if (currentGamepad1.b && !previousGamepad1.b) {
                swerveDrive.setModuleStates(new SwerveModuleState(driveSpeed, new Rotation2d(0)));
            }
            if (currentGamepad1.x && !previousGamepad1.x) {
                swerveDrive.setModuleStates(new SwerveModuleState(driveSpeed, new Rotation2d(0)));
            }
            if (currentGamepad1.y && !previousGamepad1.y) {
                swerveDrive.setModuleStates(new SwerveModuleState(driveSpeed, new Rotation2d(0)));
            }

            telemetry.addData("CommandVel", driveSpeed);
            telemetry.addData("OpenLoop", swerveDrive.openLoop);
            telemetry.addData("FLDriveVel", swerveDrive.modules[0].getState().speedMetersPerSecond);
            telemetry.addData("FRDriveVel", swerveDrive.modules[1].getState().speedMetersPerSecond);
            telemetry.addData("BLDriveVel", swerveDrive.modules[2].getState().speedMetersPerSecond);
            telemetry.addData("BRDriveVel", swerveDrive.modules[3].getState().speedMetersPerSecond);
            telemetry.update();


        }
    }

}






