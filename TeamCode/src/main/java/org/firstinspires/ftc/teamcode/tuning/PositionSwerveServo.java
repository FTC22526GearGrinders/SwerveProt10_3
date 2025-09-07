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

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.PIDControllerFRC;
import org.firstinspires.ftc.teamcode.drive.SwerveDriveConstants;
import org.firstinspires.ftc.teamcode.utils.RollingAverage;

/*
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "Concept: Position Servo", group = "Tune")
@Config
//@Disabled
public class PositionSwerveServo extends LinearOpMode {
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    public static double DEADBANDVOLTS = .001;

    static double targetAngle = 10;

    public static double KPVolts = .006;
    public static double KIVolts = .0;
    public static double KDVolts = .001;

    public PIDControllerFRC voltsController;

    public RollingAverage ra = new RollingAverage(10);


    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position


    // Define class members
    Servo servo;
    public AnalogInput servoPotentiometer;
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    private double positionError;
    private double voltsError;
    private double voltsAverage;
    private double targetVolts = 1;
    public double targetAngleWrapped;

    private int modSel;
    private int numberOfMdules = 2;


    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servo = hardwareMap.get(Servo.class, "angleServo1");
        servoPotentiometer = hardwareMap.get(AnalogInput.class, "angleInput1");
        voltsController = new PIDControllerFRC(KPVolts, KIVolts, KDVolts);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        servo.setDirection(Servo.Direction.REVERSE);
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.a && !previousGamepad1.a) {
                modSel++;
                if (modSel > numberOfMdules - 1) modSel = 0;
            }

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                targetAngle = -179;
            }

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                targetAngle = 179;
            }

            if (currentGamepad1.b) {
                targetAngle = 0;
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                targetAngle = 45;
            }

            if (currentGamepad1.y && !previousGamepad1.y) {
                targetAngle = 90;
            }

            if (currentGamepad1.dpad_left) {
                targetAngle = -100;
            }
            if (currentGamepad1.dpad_right) {
                targetAngle = -45;
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                targetAngle -= 5;

            }
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                targetAngle += 5;

            }


            targetAngleWrapped = MathUtils.clamp(targetAngle, -180, 180);

            if (targetAngleWrapped < 0) targetAngleWrapped += 180;

            targetAngleWrapped = MathUtils.clamp(targetAngleWrapped, 0.01, 180);


            voltsController.setPID(KPVolts, KIVolts, KDVolts);


            if (gamepad1.dpad_right) {
                targetAngle = 45;
            }

            if (gamepad1.dpad_left) {
                targetAngle = -45;
            }


            double volts = servoPotentiometer.getVoltage();

            ra.add(volts);

            voltsAverage = ra.getAverage();

            targetAngleWrapped = targetAngle;

            if (targetAngle < 0) targetAngleWrapped += 180;


            targetVolts = SwerveDriveConstants.voltsAtMid[0] + (1 / SwerveDriveConstants.degreesPerVolt[0]) * targetAngleWrapped;


            voltsError = targetVolts - volts;

            double voltsErrorPID = voltsController.calculate(voltsAverage, targetVolts);


            double clampedVE = MathUtils.clamp(voltsErrorPID, -.2, .2);


            // if (!atPositionVolts(DEADBANDVOLTS)) {
            position += clampedVE;


            position = MathUtils.clamp(position, 0, 1);

            // Set the servo to the new position and pause;
            servo.setPosition(position);
            //  }
            positionError = position - servo.getPosition();


            // Display the current value
            telemetry.addData("Servo Tgt Position", "%5.3f", position);
            telemetry.addData("Servo Position", "%5.3f", servo.getPosition());
            telemetry.addData("Servo Error", "%5.3f", positionError);

            telemetry.addData("Tgt Angle", "%5.3f", targetAngle);
            telemetry.addData("Tgt Angle Wrap", "%5.3f", targetAngleWrapped);

            telemetry.addData("Tgt Volts", "%5.3f", targetVolts);
            telemetry.addData("Volts", "%5.3f", volts);
            telemetry.addData("Volts Ave", "%5.3f", voltsAverage);

            telemetry.addData("VoltsError", "%5.3f", voltsError);
            telemetry.addData("VoltsErrorPID", "%5.3f", voltsErrorPID);


            telemetry.update();


            telemetry.update();
        }
    }


    private boolean atPositionVolts(double bandwidth) {
        return Math.abs(targetVolts - ra.getAverage()) <= bandwidth;
    }


}