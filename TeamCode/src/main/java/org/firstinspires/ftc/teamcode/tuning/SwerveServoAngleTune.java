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
import org.firstinspires.ftc.teamcode.drive.SwerveDriveServo;
import org.firstinspires.ftc.teamcode.utils.AverageFilter;

import Ori.Coval.Logging.AutoLogManager;
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


@TeleOp(name = "SwerveAngleTune", group = "Tune")
//@Disabled

@Config
public class SwerveServoAngleTune extends CommandOpMode {


    public int modNum = 1;
    SwerveDriveServo swerveDrive;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    FtcDashboard dashboard;
    double targetAngle;
    double driveSpeed;

    @Override
    public void initialize() {

        swerveDrive = new SwerveDriveServo(this);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        register(swerveDrive);
        KoalaLog.setup(hardwareMap);
    }

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses START)
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            AutoLogManager.periodic();
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
        }


        if (currentGamepad1.back) {
            driveSpeed = gamepad1.left_stick_y * SwerveDriveConstants.maxSpeedMetersPerSec;
            swerveDrive.modules[0].driveMotor.setPower(driveSpeed);
            swerveDrive.modules[1].driveMotor.setPower(driveSpeed);
            swerveDrive.modules[2].driveMotor.setPower(driveSpeed);
            swerveDrive.modules[3].driveMotor.setPower(driveSpeed);
        } else {
            driveSpeed = 0;
            swerveDrive.modules[0].driveMotor.setPower(driveSpeed);
            swerveDrive.modules[1].driveMotor.setPower(driveSpeed);
            swerveDrive.modules[2].driveMotor.setPower(driveSpeed);
            swerveDrive.modules[3].driveMotor.setPower(driveSpeed);
        }

        if (currentGamepad1.a) {
            targetAngle = 0;
        }
        if (currentGamepad1.b) {
            targetAngle = 90;
        }
        if (currentGamepad1.x && !previousGamepad1.x) {
            targetAngle = -90;
        }

        if (currentGamepad1.y && !previousGamepad1.y) {
            targetAngle = -110;
        }

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            targetAngle = 110;
        }
        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            targetAngle = 70;
        }
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            targetAngle = -70;
        }
        if (!currentGamepad1.dpad_up && previousGamepad1.dpad_up) {
            targetAngle = -45;
        }


        if (!currentGamepad1.dpad_down && previousGamepad1.dpad_down) {
            targetAngle = 45;
        }

        swerveDrive.setModuleStates(new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(targetAngle)));


        telemetry.addData("Target Angle", targetAngle);

        telemetry.addData("PotAngle0", swerveDrive.modules[0].getWheelAngleDeg());
        telemetry.addData("AngleServo0", swerveDrive.modules[0].getServoAngleDegrees());

        telemetry.addData("PotAngle1", swerveDrive.modules[1].getWheelAngleDeg());
        telemetry.addData("AngleServo1", swerveDrive.modules[1].getServoAngleDegrees());


        telemetry.addData("PotAngle2", swerveDrive.modules[2].getWheelAngleDeg());
        telemetry.addData("AngleServo2", swerveDrive.modules[2].getServoAngleDegrees());

        telemetry.addData("PotAngle3", swerveDrive.modules[3].getWheelAngleDeg());
        telemetry.addData("AngleServo3", swerveDrive.modules[3].getServoAngleDegrees());


        telemetry.update();
    }
}


