package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;

import org.firstinspires.ftc.teamcode.utils.Units;

public final class SwerveDriveConstants {
    public static final double WHEEL_CIRCUMFERENCE_INCHES = 1.53 * Math.PI;
    public static final double TICKS_PER_REVOLUTION = 145.1;

    public static final double maxMotorRevsPerSecond = 1150 / 60;
    public static final double TICKS_PER_INCH = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_INCHES;//approx 29
    public static final double TICKS_PER_METER = TICKS_PER_INCH * 39.37;//1160


    public static final double maxSpeedMetersPerSec = Units.inchesToMeters(maxMotorRevsPerSecond * WHEEL_CIRCUMFERENCE_INCHES);//approx 5 mps

    public static final double maxRadiansPerSecond = Math.PI / (.84); //servo specs .14 secs per 60 degrees 4.8 volts approx 3.5

    public static final double trackWidth = 11.5;
    public static final double wheelBase = 12.5;

    public static final double trackWidthMeters = Units.inchesToMeters(trackWidth);
    public static final double wheelBaseMeters = Units.inchesToMeters(wheelBase);

    public static final Translation2d flModuleOffset = new Translation2d(wheelBaseMeters / 2.0,
            trackWidthMeters / 2.0);
    public static final Translation2d frModuleOffset = new Translation2d(wheelBaseMeters / 2.0,
            -trackWidthMeters / 2.0);
    public static final Translation2d blModuleOffset = new Translation2d(-wheelBaseMeters / 2.0,
            trackWidthMeters / 2.0);
    public static final Translation2d brModuleOffset = new Translation2d(-wheelBaseMeters / 2.0,
            -trackWidthMeters / 2.0);

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(flModuleOffset, frModuleOffset, blModuleOffset, brModuleOffset);


    //For controlling the robot Values can be adjusted depending on who is controlling
    public static final double rotationMultiplier = 0.5;

}
