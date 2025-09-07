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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SwerveDriveConstants;
import org.firstinspires.ftc.teamcode.drive.SwerveDriveServo;

import Ori.Coval.Logging.Logger.KoalaLog;



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


@TeleOp(name = "ServoSwerveAngleJog", group = "Tune")
//@Disabled

@Config
public class ServoSwerveAngleJog extends CommandOpMode {

    // Declare OpMode members.


    public int modNum = 1;
    SwerveDriveServo swerveDrive;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    FtcDashboard dashboard;
    double targetAngle = 0.5;

    double targetIncrement = 0.001;

    int lpctr;

    int modSel;

    double driveSpeed;

    int numberOfModules = 4;


    @Override
    public void initialize() {
        swerveDrive = new SwerveDriveServo(this);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        register(swerveDrive);
        KoalaLog.setup(hardwareMap);
        // new ServoDriveCommand(swerveDrive, currentGamepad1, false, this).schedule();
    }

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses START)
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            run();
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);


            swerveDrive.modules[modSel].angleServo.setPosition(targetAngle);
//            swerveDrive.modules[1].angleServo.setPosition(targetAngle);
//            swerveDrive.modules[2].angleServo.setPosition(targetAngle);
//            swerveDrive.modules[3].angleServo.setPosition(targetAngle);


//                driveSpeed = gamepad1.left_stick_y * SwerveDriveConstants.maxSpeedMetersPerSec;
//                swerveDrive.modules[0].driveMotor.setPower(driveSpeed);
//                swerveDrive.modules[1].driveMotor.setPower(driveSpeed);
//                swerveDrive.modules[2].driveMotor.setPower(driveSpeed);
//                swerveDrive.modules[3].driveMotor.setPower(driveSpeed);
//            } else {
//                driveSpeed = 0;
//                swerveDrive.modules[0].driveMotor.setPower(driveSpeed);
//                swerveDrive.modules[1].driveMotor.setPower(driveSpeed);
//                swerveDrive.modules[2].driveMotor.setPower(driveSpeed);
//                swerveDrive.modules[3].driveMotor.setPower(driveSpeed);
//            }


            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                targetAngle = .6;
                if (targetAngle > SwerveDriveConstants.servoRangeSetting)
                    targetAngle -= SwerveDriveConstants.servoRangeSetting;
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                targetAngle = .1;
            }


            if (currentGamepad1.a && !previousGamepad1.a) {
                modSel++;
                if (modSel > numberOfModules - 1) modSel = 0;
            }

            if (currentGamepad1.b && !previousGamepad1.b) {
                if (targetIncrement == .01)
                    targetIncrement = .001;
                else targetIncrement = .01;
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                targetAngle = 0.25;
            }

            if (currentGamepad1.y && !previousGamepad1.y) {
                targetAngle = .5;
            }

            if (currentGamepad1.dpad_left) {
                targetAngle = 0.01;
            }
            if (currentGamepad1.dpad_right) {
                targetAngle = .99;
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                targetAngle -= targetIncrement;
                if (targetAngle > 90) targetAngle -= SwerveDriveConstants.servoRangeSetting;
            }
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                targetAngle += targetIncrement;
            }

            lpctr++;


            telemetry.addData("ModuleSel", modSel);
            telemetry.addData("TargetInc", "%5.3f", targetIncrement);
            telemetry.addData("Target", "%5.3f", targetAngle);
            telemetry.addData("Offset", "%5.3f",  targetAngle-.5);

            telemetry.addData("PotVolts", "%5.3f", swerveDrive.modules[modSel].getPotVolts());
            telemetry.addData("PotVoltsAve", "%5.3f", swerveDrive.modules[modSel].getPotVoltsAve());
            telemetry.addData("ServoCmd", "%5.3f", swerveDrive.modules[modSel].getServoPosition());
            telemetry.addData("AngleFromPot", "%5.3f", swerveDrive.modules[modSel].getWheelAngleDeg());
            telemetry.addData("Speed", "%5.3f", swerveDrive.modules[modSel].getState().speedMetersPerSecond);


            telemetry.update();

        }
    }

}



