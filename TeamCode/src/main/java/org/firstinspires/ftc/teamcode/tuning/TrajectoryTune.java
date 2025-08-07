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
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SwerveDrive;
import org.firstinspires.ftc.teamcode.utils.Drawing;
import org.firstinspires.ftc.teamcode.utils.Trajectories;


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


@TeleOp(name = "TrajectoryTune", group = "Tune")
//@Disabled
@Config
public class TrajectoryTune extends CommandOpMode {


    public static double XKP = 1.;
    public static double XKI = .01;
    public static double XKD = .01;

    public static double YKP = 1.;
    public static double YKI = .0;
    public static double YKD = .0;

    public static double THETAKP = .01;
    public static double THETAKI = .0;
    public static double THETAKD = .0;

    Trajectories m_trajectories;
    SwerveDrive swerveDrive;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void initialize() {
        swerveDrive = new SwerveDrive(this);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    }

    @Override
    public void runOpMode() {


        // Wait for the game to start (driver presses START)
        waitForStart();

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), swerveDrive.getPose());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                swerveDrive.setXControllerKvals(XKP, XKI, XKD);
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                swerveDrive.setYControllerKvals(YKP, YKI, YKD);
            }
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                swerveDrive.setThetaControllerKvals(THETAKP, THETAKI, THETAKD);
            }


            if (currentGamepad1.dpad_up) {
                swerveDrive.setModuleOpenloop(true);
            }

            if (currentGamepad1.dpad_down) {
                swerveDrive.setModuleOpenloop(false);
            }


            if (currentGamepad1.a)
                swerveDrive.runTrajectory(m_trajectories.leftStart);

            if (currentGamepad1.b && !previousGamepad1.b) {

                swerveDrive.runTrajectory(m_trajectories.leftStart);
            }
            if (currentGamepad1.x && !previousGamepad1.x) {
                swerveDrive.runTrajectory(m_trajectories.leftStart);
            }
            if (currentGamepad1.y && !previousGamepad1.y) {
                swerveDrive.runTrajectory(m_trajectories.leftStart);
            }

            telemetry.addData("OpenLoop", swerveDrive.openLoop);
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






