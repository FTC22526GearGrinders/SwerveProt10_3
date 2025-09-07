package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.utils.Units;

public final class SwerveDriveConstants {
    public static final double WHEEL_CIRCUMFERENCE_INCHES = 1.53 * Math.PI;//approx 5
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_CIRCUMFERENCE_INCHES);//approx .12
    //    public static final double TICKS_PER_INCH = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_INCHES;//approx 29
    public static final double TICKS_PER_REVOLUTION = 145.1;
    public static final double maxMotorRevsPerSecond = 1150. / 60.;//19
    public static final double maxAngleRevsPerSecond = 1. / 6 * (.14); //servo specs .14 secs per 60 degrees 4.8 volts .84 sec per rev = 1.2 revs per sec

    public static final double TICKS_PER_METER = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_METERS;//approx 1200
    public static final double maxSpeedMetersPerSec = 2.4;// maxMotorRevsPerSecond * WHEEL_CIRCUMFERENCE_METERS;//approx 2.4 mps
    public static final double maxAccelerationMPSPS = maxSpeedMetersPerSec * 2;
    public static double calcKV = 12 / maxSpeedMetersPerSec;//approx 5

    public static final double maxAngleRadiansPerSecond = maxAngleRevsPerSecond * 2. * Math.PI;

    public static final double trackWidth = 15.375;
    public static final double wheelBase = 10.5;

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

    public static double robotLength;

    public static double robotWidth;

    public static double fieldLength = Units.inchesToMeters(72.);
    public static double fieldWidth = Units.inchesToMeters(72.);
    public static double maxAngularVelocity = 4;


    //module angle servo values

    public static double servoRangeSetting = 250;
    public static double minServoPosition = .01;
    public static double maxServoPosition = .99;
    public static double midServoPosition = .5;
    public static double[] voltsAtMin = {.826, .60, .640, .640};
    public static double[] voltsAtMax = {2.492, 2.688, 2.64, 2.66};
    public static double[] voltsAtMid = {1.626, 1.643, 1.6, 1.6};

    public static double degreesPerVolt[] =

            {180 / (voltsAtMax[0] - voltsAtMin[0]), 180 / (voltsAtMax[1] - voltsAtMin[1]), 180 / (voltsAtMax[2] - voltsAtMin[2]), 180 / (voltsAtMax[3] - voltsAtMin[3])};


    public static double minAngleDegrees = -servoRangeSetting / 2;
    public static double maxAngleDegrees = servoRangeSetting / 2;
    public static double midAngleDegrees = 0;

    public static double degreeRange = maxAngleDegrees - minAngleDegrees;//

    public static double servoPositionPerDegree = 1. / servoRangeSetting;


}



