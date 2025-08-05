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


@TeleOp(name = "SwerveAngleTune", group = "Tune")
//@Disabled
@Config
public class SwerveAngleTune extends CommandOpMode {

    // Declare OpMode members.

    public static double[] ANGLEKP = {.01, .01, .01, .01};
    public static double[] ANGLEKI = {.0, .0, .0, .0};
    public static double[] ANGLEKD = {.0, .0, .0, .0};

    SwerveDrive swerveDrive;

    @Override
    public void initialize() {

    }

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        // Wait for the game to start (driver presses START)
        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                for (int i = 0; i < swerveDrive.modules.length; i++) {
                    swerveDrive.setModuleAngleKP(i, ANGLEKP[i]);
                    swerveDrive.setModuleAngleKI(i, ANGLEKI[i]);
                    swerveDrive.setModuleAngleKD(i, ANGLEKD[i]);
                }
            }
            double driveSpeed = gamepad1.left_stick_y * SwerveDriveConstants.maxSpeedMetersPerSec;
            if (currentGamepad1.a && !previousGamepad1.a) {
                swerveDrive.setModuleStates(new SwerveModuleState(driveSpeed, new Rotation2d(0)));
            }
            if (currentGamepad1.b && !previousGamepad1.b) {
                swerveDrive.setModuleStates(new SwerveModuleState(driveSpeed, new Rotation2d(Math.PI / 2)));
            }
            if (currentGamepad1.x && !previousGamepad1.x) {
                swerveDrive.setModuleStates(new SwerveModuleState(driveSpeed, new Rotation2d(Math.PI)));
            }
            if (currentGamepad1.y && !previousGamepad1.y) {
                swerveDrive.setModuleStates(new SwerveModuleState(driveSpeed, new Rotation2d(-Math.PI / 2)));
            }

            telemetry.addData("FLAnglePos", swerveDrive.modules[0].getState().angle);
            telemetry.addData("FRAnglePos", swerveDrive.modules[1].getState().angle);
            telemetry.addData("BLAnglePos", swerveDrive.modules[2].getState().angle);
            telemetry.addData("BRAnglePos", swerveDrive.modules[3].getState().angle);
            telemetry.addData("FLAngleVel", swerveDrive.modules[0].getState().speedMetersPerSecond);
            telemetry.addData("FRAngleVel", swerveDrive.modules[1].getState().speedMetersPerSecond);
            telemetry.addData("BLAngleVel", swerveDrive.modules[2].getState().speedMetersPerSecond);
            telemetry.addData("BRAngleVel", swerveDrive.modules[3].getState().speedMetersPerSecond);

            telemetry.update();


        }
    }

}







