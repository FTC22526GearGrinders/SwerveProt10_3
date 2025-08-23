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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SwerveModule;
import org.firstinspires.ftc.teamcode.drive.SwerveModuleConfig;
import org.firstinspires.ftc.teamcode.drive.SwerveModuleServo;


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


@TeleOp(name = "SingleSwerveAngleTune", group = "Tune1")
//@Disabled
@Config
public class SingleSwerveServoAngleTune extends CommandOpMode {

    // Declare OpMode members.

    public static double ANGLEKP = .01;
    public static double ANGLEKI = 0;
    public static double ANGLEKD = 0;
    public static double ANGLE_OFFSET = -65.5;

    SwerveModuleConfig mod;
    SwerveModuleServo module;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    double driveSpeed;
    double targetAngle;

    double maxRange = 230;

    @Override
    public void initialize() {
        mod = new SwerveModuleConfig(0,
                "driveMotor2", "angleServo2", "angleInput2", ANGLE_OFFSET, DcMotorSimple.Direction.REVERSE);
        module = new SwerveModuleServo(mod, this);
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
                module.setAngleKP(ANGLEKP);
                module.setAngleKI(ANGLEKI);
                module.setAngleKD(ANGLEKD);
            }
            /*
             * total range is 230
             * command range is 0-1
             * we want command +-115 target angle
             * so 0 deg = .5  115=1 -115=0
             * servo command = fun(targetAngle)
             * servo inc = 1/230
             * if tgt =0 cmd = .5 +targetAngle/range
             *
             * */

            module.setAngleDegrees(
                    targetAngle);

            if (currentGamepad1.a) {
                targetAngle = 0;
            }
            if (currentGamepad1.b) {
                targetAngle = 60;
            }
            if (currentGamepad1.x) {
                targetAngle = -60;
            }
            if (currentGamepad1.y) {
                targetAngle = 90;
            }

            if (currentGamepad1.dpad_left) {
                targetAngle = -90;
            }
            if (currentGamepad1.dpad_right) {
                // targetAngle = -.25;
            }
            if (currentGamepad1.dpad_up) {
                //  targetAngle = -.5;
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                targetAngle += 1;
            }


            double pot = module.getWheelAngleDeg();

            telemetry.addData("AnglePos", module.angleServo.getPosition() * maxRange);
            telemetry.addData("AngleTgt", targetAngle);
            telemetry.addData("AnglePot", pot);


            telemetry.update();

        }
    }

}






