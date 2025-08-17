package org.firstinspires.ftc.teamcode.simulation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SwerveDriveConstants;
import org.firstinspires.ftc.teamcode.drive.SwerveModuleConfig;
import org.firstinspires.ftc.teamcode.utils.Units;


public class SwerveDriveSim extends SubsystemBase {

    public final SwerveModuleSim[] modules;
    final ElapsedTime timer = new ElapsedTime();
    private final SwerveDriveOdometry m_odometry;

    public Telemetry telemetry;
    double previousTimeSecs;
    private Pose2d robotPose = new Pose2d();
    private Rotation2d simYaw = new Rotation2d();
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private double startTime;

    public SwerveDriveSim(CommandOpMode opMode) {
        SwerveModuleConfig fl = new SwerveModuleConfig(0,
                "driveMotor1", "angleServo1", "angleInput1", 117.27, DcMotorSimple.Direction.REVERSE);

        SwerveModuleConfig fr = new SwerveModuleConfig(1,
                "driveMotor2", "angleServo2", "angleInput2", -141.38, DcMotorSimple.Direction.REVERSE);

        SwerveModuleConfig bl = new SwerveModuleConfig(2,
                "driveMotor3", "angleServo3", "angleInput3", -129.16, DcMotorSimple.Direction.REVERSE);

        SwerveModuleConfig br = new SwerveModuleConfig(3,
                "driveMotor4", "angleServo4", "angleInput4", -50.8, DcMotorSimple.Direction.REVERSE);

        modules = new SwerveModuleSim[]{
                new SwerveModuleSim(fl, opMode),
                new SwerveModuleSim(fr, opMode),
                new SwerveModuleSim(bl, opMode),
                new SwerveModuleSim(br, opMode)
        };


        m_odometry = new SwerveDriveOdometry(SwerveDriveConstants.swerveKinematics, getHeading());

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

        timer.reset();
    }


    public void drive(double translation, double strafe, double rotation, boolean fieldRelative) {

        double new_translation = translation * SwerveDriveConstants.maxSpeedMetersPerSec;
        double new_strafe = strafe * SwerveDriveConstants.maxSpeedMetersPerSec;
        double new_rotation = rotation * SwerveDriveConstants.maxAngleRadiansPerSecond * SwerveDriveConstants.rotationMultiplier;

        speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(new_translation, new_strafe, new_rotation, getHeading())
                : new ChassisSpeeds(new_translation, new_strafe, new_rotation);


        //----------I'm not sure if any of this is needed------------copied from FRC--------------
//        double currentTimeStamp = (double) System.nanoTime() / 1E9; //to seconds
//        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
//        period = currentTimeStamp - lastTimeStamp;
//        if (period == 0) period = 1/40;
//
//        lastTimeStamp = currentTimeStamp;
//
//        speeds = discretize(speeds.vxMetersPerSecond,
//                speeds.vyMetersPerSecond,
//                speeds.omegaRadiansPerSecond,
//                period);
        //---------------------------------------------------------------------------------------

        SwerveModuleState[] states = SwerveDriveConstants.swerveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.normalizeWheelSpeeds(states, SwerveDriveConstants.maxSpeedMetersPerSec);

        for (SwerveModuleSim module : modules) {
            module.setState(states[module.moduleNumber]);
        }
    }

    public void setModuleStates(SwerveModuleState state) {
        for (SwerveModuleSim module : modules) {
            module.setState(state);
        }
    }


    public Rotation2d getHeading() {
        return simYaw;
    }

    public void resetYaw() {
        simYaw = new Rotation2d();
    }

    public ChassisSpeeds discretize( //Imported Directly From WPILIB not sure if necessary
                                     double vxMetersPerSecond,
                                     double vyMetersPerSecond,
                                     double omegaRadiansPerSecond,
                                     double dtSeconds) {

        Pose2d desiredDeltaPose =
                new Pose2d(
                        vxMetersPerSecond * dtSeconds,
                        vyMetersPerSecond * dtSeconds,
                        new Rotation2d(omegaRadiansPerSecond * dtSeconds));


        Twist2d twist = new Pose2d().log(desiredDeltaPose);

        return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
    }

    public Pose2d getPose() {
        return robotPose;
    }

    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(pose,getHeading());
    }

    public Pose2d converMetersPoseToInches(Pose2d pose) {
        double x = Units.metersToInches(pose.getX());
        double y = Units.metersToInches(pose.getY());
        Rotation2d r2d = pose.getRotation();
        telemetry.addData("R2D", r2d.getDegrees());
        return new Pose2d(x, y, r2d);
    }


    public Pose2d convertInchesPoseToMeters(Pose2d pose) {
        double x = Units.inchesToMeters(pose.getX());
        double y = Units.inchesToMeters(pose.getY());
        Rotation2d r2d = pose.getRotation();
        return new Pose2d(x, y, r2d);
    }


    public void updateSimYaw() {
        double radsChange = speeds.omegaRadiansPerSecond * .02;
        double newRads = simYaw.getRadians() + radsChange;

        newRads %= 2 * Math.PI;
        newRads = newRads < 0 ? newRads + 2 * Math.PI : newRads;

        simYaw = new Rotation2d(newRads);
    }


    @Override
    public void periodic() {
        updateSimYaw();
        robotPose = updateOdometry();


//        telemetry.addData("Heading", simYaw.getDegrees());
//        telemetry.addData("RotSpeed", speeds.omegaRadiansPerSecond);
//        telemetry.addData("X", robotPose.getX());
//        telemetry.addData("Y", robotPose.getY());
//        telemetry.addData("THETA", robotPose.getHeading());

    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] temp = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++) {
            temp[i] = modules[i].getState();
        }
        return temp;
    }


    public Pose2d updateOdometry() {
        double t = timer.time();
        double tinc = t - previousTimeSecs;
        previousTimeSecs = t;
        //  showTimeValues(tinc);
        return m_odometry.updateWithTime(timer.time(), getHeading(), getStates());
    }


    public void showTimeValues(double tinc) {

        telemetry.addData("TIMER", timer.time());

        telemetry.addData("TIMERINC", tinc);

        telemetry.update();


    }

}
