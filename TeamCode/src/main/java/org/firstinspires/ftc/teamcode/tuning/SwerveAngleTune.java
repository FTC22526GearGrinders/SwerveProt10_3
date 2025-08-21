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
public class SwerveAngleTune extends CommandOpMode {

    // Declare OpMode members.

    public static double[] ANGLEKP = {.015, .015, .015, .015};
    public static double[] ANGLEKI = {.00, .0, .0, .0};
    public static double[] ANGLEKD = {0., .0, .0, .0};
    public static double MAX_ANGLE_PID = .2;
    public static boolean ALL_SAME = true;
    public static double SERVO_ZERO_OFFSET = .005;
    public static double SERVO_POWER_SET = 0;
    public int modNum = 1;
    SwerveDrive swerveDrive;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    FtcDashboard dashboard;
    double targetAngle;
    double driveSpeed;
    private AverageFilter aveFilter = new AverageFilter();

    @Override
    public void initialize() {

        swerveDrive = new SwerveDrive(this);
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

            if (ALL_SAME && (ANGLEKP[0] != ANGLEKP[1] || ANGLEKI[0] != ANGLEKI[1] || ANGLEKD[0] != ANGLEKD[1])) {
                for (int i = 1; i < swerveDrive.modules.length; i++) {
                    ANGLEKP[i] = ANGLEKP[0];
                    ANGLEKI[i] = ANGLEKI[0];
                    ANGLEKD[i] = ANGLEKD[0];
                }
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                for (int i = 0; i < swerveDrive.modules.length; i++) {
                    swerveDrive.setModuleThetaAngleKP(i, ANGLEKP[i]);
                    swerveDrive.setModuleThetaAngleKI(i, ANGLEKI[i]);
                    swerveDrive.setModuleAngleKD(i, ANGLEKD[i]);
                    swerveDrive.setMAX_ANGLE_PID(i, MAX_ANGLE_PID);
                }
            }


             //   swerveDrive.setModuleStates(new SwerveModuleState(driveSpeed, Rotation2d.fromDegrees(targetAngle)));


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
                targetAngle = 80;
            }
            if (currentGamepad1.x && !previousGamepad1.x) {
                SERVO_POWER_SET -= .01;
            }

            if (currentGamepad1.y && !previousGamepad1.y) {
                SERVO_POWER_SET += 0.01;
            }
            
            if (currentGamepad1.dpad_left) {
                targetAngle = 45;
            }
            if (currentGamepad1.dpad_right) {
                targetAngle = 70;
            }
            if (currentGamepad1.dpad_up) {
                for (int i = 0; i < swerveDrive.modules.length; i++) {
                    swerveDrive.setModuleAnglePower(i, SERVO_POWER_SET + SERVO_ZERO_OFFSET);
                }
            }
            if (!currentGamepad1.dpad_up && previousGamepad1.dpad_up) {
                for (int i = 0; i < swerveDrive.modules.length; i++) {
                    swerveDrive.setModuleAnglePower(i, 0);
                }
            }

            if (currentGamepad1.dpad_down) {
                for (int i = 0; i < swerveDrive.modules.length; i++) {
                    swerveDrive.setModuleAnglePower(i, -SERVO_POWER_SET + SERVO_ZERO_OFFSET);
                }
            }
            if (!currentGamepad1.dpad_down && previousGamepad1.dpad_down) {
                for (int i = 0; i < swerveDrive.modules.length; i++) {
                    swerveDrive.setModuleAnglePower(i, 0);
                }
            }

            boolean log = false;

            double wheelDegs = swerveDrive.modules[modNum].getWheelAngleDeg();
            double millisec = swerveDrive.modules[modNum].potTimer.milliseconds();
            double power = swerveDrive.modules[modNum].getServoPower();
            if (log) {
                KoalaLog.log("Angle " + String.valueOf(modNum), wheelDegs, true);
//                KoalaLog.log("Angle1", swerveDrive.modules[1].getState().angle.getDegrees(), true);
//                KoalaLog.log("Angle2", swerveDrive.modules[2].getState().angle.getDegrees(), true);
//                KoalaLog.log("Angle3", swerveDrive.modules[3].getState().angle.getDegrees(), true);
                KoalaLog.log("Power " + String.valueOf(modNum), power, true);
                KoalaLog.log("Millisec", millisec, true);
//                KoalaLog.log("Power1", swerveDrive.modules[1].getServoPower(), true);
//                KoalaLog.log("Power2", swerveDrive.modules[2].getServoPower(), true);
//                KoalaLog.log("Power3", swerveDrive.modules[3].getServoPower(), true);
                telemetry.update();

            } else {
                telemetry.addData("Target Angle", targetAngle);

                telemetry.addData("Angle0", swerveDrive.modules[0].getWheelAngleDeg());
                telemetry.addData("Power0", swerveDrive.modules[0].getServoPower());

                telemetry.addData("Angle1", swerveDrive.modules[1].getWheelAngleDeg());
                telemetry.addData("Power1", swerveDrive.modules[1].getServoPower());

                telemetry.addData("Angle2", swerveDrive.modules[2].getWheelAngleDeg());
                telemetry.addData("Power2", swerveDrive.modules[2].getServoPower());

                telemetry.addData("Angle3", swerveDrive.modules[3].getWheelAngleDeg());
                telemetry.addData("Power3", swerveDrive.modules[3].getServoPower());











//                double verr = swerveDrive.modules[1].thetapidController.getVelocityError();
//
//                double perr = swerveDrive.modules[1].thetapidController.getPositionError();
//
//                telemetry.addData("VERR1", verr);
//
//                telemetry.addData("PERR1", perr);


                telemetry.update();
            }
        }
    }
}


