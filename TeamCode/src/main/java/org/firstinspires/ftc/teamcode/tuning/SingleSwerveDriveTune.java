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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SwerveDriveConstants;
import org.firstinspires.ftc.teamcode.drive.SwerveModule;
import org.firstinspires.ftc.teamcode.drive.SwerveModuleConfig;


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


@TeleOp(name = "SingleDriveTune", group = "Tune1")
//@Disabled
@Config
public class SingleSwerveDriveTune extends CommandOpMode {

    public static double DRIVEKP = .01;
    public static double DRIVEKI = .0;
    public static double DRIVEKD = .0;
    public static double DRIVEKS = .01;
    public static double DRIVEKV = SwerveDriveConstants.calcKV;
    public static double DRIVEKA = .0;
    public static boolean OPENLOOP = true;
    public static double ANGLE_OFFSET = 0;
    SwerveModuleConfig mod;
    SwerveModule module;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    double driveSpeed;
    double finalDriveSpeed;

    @Override
    public void initialize() {
        mod = new SwerveModuleConfig(0,
                "driveMotor", "angleServo", "angleInput", ANGLE_OFFSET, DcMotorSimple.Direction.REVERSE);

        module = new SwerveModule(mod, this);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    }

    @Override
    public void runOpMode() {

        initialize();
        // Wait for the game to start (driver presses START)
        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                module.setDriveKP(DRIVEKP);
                module.setDriveKI(DRIVEKI);
                module.setDriveKD(DRIVEKD);
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                module.createFeedForward(DRIVEKS, DRIVEKV, DRIVEKA);
            }

            if (currentGamepad1.dpad_right)
                finalDriveSpeed = -driveSpeed;

            if (currentGamepad1.dpad_left)
                finalDriveSpeed = driveSpeed;

            if (!currentGamepad1.dpad_left && !currentGamepad1.dpad_right)
                finalDriveSpeed = 0;

            module.setState(new SwerveModuleState(finalDriveSpeed, new Rotation2d(0)));


            if (currentGamepad1.dpad_up) {
                module.setOpenLoop(true);
            }

            if (currentGamepad1.dpad_down) {
                module.setOpenLoop(false);
            }

            if (currentGamepad1.a && !previousGamepad1.a) {
                driveSpeed = 0.5;
            }
            if (currentGamepad1.b && !previousGamepad1.b) {
                driveSpeed = 0.75;
            }
            if (currentGamepad1.x && !previousGamepad1.x) {
                driveSpeed = 1;
            }
            if (currentGamepad1.y && !previousGamepad1.y) {
                driveSpeed = 1.25;
            }

            telemetry.addData("CommandVel", driveSpeed);
            telemetry.addData("OpenLoop", OPENLOOP);
            telemetry.addData("driveMotorVel", module.driveMotor.getVelocity());
            telemetry.addData("DriveVel", module.getState().speedMetersPerSecond);

            telemetry.update();


        }
    }
}







