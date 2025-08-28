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
        double rot = gamepad.right_stick_x;
        rot=0;
double y =-gamepad.left_stick_y / speedDivisor;
y=0;
        drive.drive(
                y,
                gamepad.left_stick_x / speedDivisor,
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