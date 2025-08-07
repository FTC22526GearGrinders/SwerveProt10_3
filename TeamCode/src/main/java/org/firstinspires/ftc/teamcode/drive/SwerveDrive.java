package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.HolonomicDriveController;



public class SwerveDrive extends SubsystemBase {
    public final SwerveModule[] modules;
    final ElapsedTime timer = new ElapsedTime();
    private final BHI260IMU imu;
    private final SwerveDriveOdometry m_odometry;
    public Telemetry telemetry;
    public boolean openLoop;
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            1, 2);
    private final PIDController xpidController = new PIDController(.01, 0, 0);
    private final PIDController ypidController = new PIDController(.01, 0, 0);
    private final ProfiledPIDController thetapidController = new ProfiledPIDController(.01, 0, 0, constraints);
    private final HolonomicDriveController m_holonomicController
            = new HolonomicDriveController(
            xpidController,
            ypidController,
            thetapidController);

    private Pose2d robotPose = new Pose2d();

    public SwerveDrive(CommandOpMode opMode) {
        SwerveModuleConfig fl = new SwerveModuleConfig(0,
                "driveMotor1", "angleServo1", "angleInput1", 117.27, DcMotorSimple.Direction.REVERSE);

        SwerveModuleConfig fr = new SwerveModuleConfig(1,
                "driveMotor2", "angleServo2", "angleInput2", -141.38, DcMotorSimple.Direction.REVERSE);

        SwerveModuleConfig bl = new SwerveModuleConfig(2,
                "driveMotor3", "angleServo3", "angleInput3", -129.16, DcMotorSimple.Direction.REVERSE);

        SwerveModuleConfig br = new SwerveModuleConfig(3,
                "driveMotor4", "angleServo4", "angleInput4", -50.8, DcMotorSimple.Direction.REVERSE);

        modules = new SwerveModule[]{
                new SwerveModule(fl, opMode),
                new SwerveModule(fr, opMode),
                new SwerveModule(bl, opMode),
                new SwerveModule(br, opMode)
        };

        imu = opMode.hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                        )
                )
        );

        m_odometry = new SwerveDriveOdometry(SwerveDriveConstants.swerveKinematics, getHeading());

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
    }


    public void drive(double translation, double strafe, double rotation, boolean fieldRelative) {

        double new_translation = translation * SwerveDriveConstants.maxSpeedMetersPerSec;
        double new_strafe = strafe * SwerveDriveConstants.maxSpeedMetersPerSec;
        double new_rotation = rotation * SwerveDriveConstants.maxRadiansPerSecond * SwerveDriveConstants.rotationMultiplier;

        ChassisSpeeds speeds = fieldRelative
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

        for (SwerveModule module : modules) {
            module.setState(states[module.moduleNumber]);
        }
    }

    public void setModuleStates(SwerveModuleState state) {
        for (SwerveModule module : modules) {
            module.setState(state);
        }
    }

    public void setModuleOpenloop(boolean val) {
        for (SwerveModule module : modules) {
            module.setOpenLoop(val);
        }
        openLoop = val;
    }


    public Rotation2d getHeading() {
        return new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    public void resetYaw() {
        imu.resetYaw();
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
        return new Pose2d();
    }


    @Override
    public void periodic() {

        robotPose = updateOdometry();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Heading", angles.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Roll", angles.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Pitch", angles.getPitch(AngleUnit.DEGREES));
        telemetry.addData("TPI", SwerveDriveConstants.TICKS_PER_INCH);
        telemetry.addData("TPM", SwerveDriveConstants.TICKS_PER_METER);

        telemetry.addData("X", robotPose.getX());
        telemetry.addData("Y", robotPose.getY());

    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] temp = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++) {
            temp[i] = modules[i].getState();
        }
        return temp;
    }


    public Pose2d updateOdometry() {
        return m_odometry.updateWithTime(System.currentTimeMillis() / 1000, getHeading(), getStates());
    }

    public void runTrajectory(Trajectory trajectory) {
        setModuleOpenloop(false);
        double curTime = timer.time();
        Trajectory.State goal = trajectory.sample(curTime);
        ChassisSpeeds adjustedSpeeds = m_holonomicController.calculate(
                getPose(), goal, goal.poseMeters.getRotation());
        SwerveModuleState[] moduleStates = SwerveDriveConstants.swerveKinematics.toSwerveModuleStates(adjustedSpeeds);
        for (SwerveModule module : modules) {
            module.setState(moduleStates[module.moduleNumber]);
        }
    }

    public double getModuleAngleKP(int module) {
        return modules[module].getAngleKP();
    }

    public void setModuleAngleKP(int module, double val) {
        modules[module].setAngleKP(val);
    }

    public double getModuleAngleKI(int module) {
        return modules[module].getAngleKI();
    }

    public void setModuleAngleKI(int module, double val) {
        modules[module].setAngleKI(val);
    }

    public double getModuleAngleKD(int module) {
        return modules[module].getAngleKD();
    }

    public void setModuleAngleKD(int module, double val) {
        modules[module].setAngleKD(val);
    }

    public double getModuleDriveKP(int module) {
        return modules[module].getDriveKP();
    }

    public void setModuleDriveKP(int module, double val) {
        modules[module].setDriveKP(val);
    }

    public double getModuleDriveKI(int module) {
        return modules[module].getDriveKI();
    }

    public void setModuleDriveKI(int module, double val) {
        modules[module].setDriveKI(val);
    }

    public double getModuleDriveKD(int module) {
        return modules[module].getDriveKD();
    }

    public void setModuleDriveKD(int module, double val) {
        modules[module].setDriveKD(val);
    }

    public void setXControllerKvals(double pval, double ival, double dval) {
        xpidController.setPID(pval, ival, dval);
    }

    public void setYControllerKvals(double pval, double ival, double dval) {
        ypidController.setPID(pval, ival, dval);
    }

    public void setThetaControllerKvals(double pval, double ival, double dval) {
        thetapidController.setPID(pval, ival, dval);
    }

    public void showAngleValues(){
        telemetry.addData("FLAngle",911);

        telemetry.update();


    }

}
