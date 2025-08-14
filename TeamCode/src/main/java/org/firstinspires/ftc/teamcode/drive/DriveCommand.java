package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;


public class DriveCommand extends CommandBase {

    private final CommandOpMode myOpmode;
    private final SwerveDrive drive;
    boolean fieldRelative;
    Gamepad gamepad;
    double speedDivisor = 2;
int  test;
    public DriveCommand(SwerveDrive drive, Gamepad gamepad, boolean fieldRelative,  CommandOpMode opMode) {
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

        drive.drive(
        -gamepad.left_stick_y / speedDivisor,
                gamepad.left_stick_x / speedDivisor,
                gamepad.right_stick_x,
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