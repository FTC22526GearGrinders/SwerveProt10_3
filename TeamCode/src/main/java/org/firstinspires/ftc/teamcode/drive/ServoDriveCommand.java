package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;


public class ServoDriveCommand extends CommandBase {

    private final CommandOpMode myOpmode;
    private final SwerveDriveServo drive;
    boolean fieldRelative;
    Gamepad gamepad;
    double speedDivisor = 2;

    double deadband = .08;
    int test;


    public ServoDriveCommand(SwerveDriveServo drive, Gamepad gamepad, boolean fieldRelative, CommandOpMode opMode) {
        this.drive = drive;
        myOpmode = opMode;
        this.gamepad = gamepad;
        this.fieldRelative = fieldRelative;
        addRequirements(this.drive);
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double translation = -gamepad.left_stick_y / speedDivisor;
        double strafe = gamepad.left_stick_x / speedDivisor;
        double rot = gamepad.right_stick_x;

        if (Math.abs(translation) < deadband) translation = 0;
        if (Math.abs(strafe) < deadband) strafe = 0;
      //  if (Math.abs(rot) < deadband) rot = 0;

        myOpmode.telemetry.addData("FWD",translation);

        myOpmode.telemetry.addData("STR",strafe);

        myOpmode.telemetry.addData("ROT",rot);


        drive.drive(
                translation,
                strafe,
                rot,
                fieldRelative);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, true);


    }

    @Override
    public boolean isFinished() {
        return false;
    }

}