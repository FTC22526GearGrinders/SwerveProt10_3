package org.firstinspires.ftc.teamcode.drive;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Units;

import Ori.Coval.Logging.Logger.KoalaLog;

//@AutoLog
public class SwerveModule extends SubsystemBase {

    private final PIDControllerFRC driveController = new PIDControllerFRC(0.01, 0, 0, .02);

    private final PIDControllerFRC angleController = new PIDControllerFRC(.015, 0.0, 0., 0.02);
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            25, 30);
    private final ProfiledPIDController thetapidController = new ProfiledPIDController(.02, 0, 0, constraints);

    private final boolean showTelemetry = false;
    private final boolean log = false;
    private final double acceLimit = 5;
    private final double powerLimit = 1;
    private final double ks = 0;
    private final double ka = 0;
    public CRServo angleServo;
    public DcMotorEx driveMotor;
    public AnalogInput servoPotentiometer;
    public double angleOffset;
    public boolean openLoop;
    public int moduleNumber;
    public Telemetry telemetry;
    public double setpoint = 0;
    public double anglePID = 0;
    public double wheelDegs = 0;
    public double pidout;
    public double feedForwardVolts;
    public SwerveModuleState newState = new SwerveModuleState();
    public SwerveModuleState origState = new SwerveModuleState();
    public double sampleCount;
    double volts;
    boolean optimized;
    private boolean lockReadPot;
    private SimpleMotorFeedforward driveFeedforward;
    private double lastSpeed;
    private double pidOut;
    private double potAngle;
    public double MAX_ANGLE_PID=.1;


    public SwerveModule(SwerveModuleConfig config, CommandOpMode opMode) {
        driveMotor = opMode.hardwareMap.get(DcMotorEx.class, config.driveMotorName);

        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        createFeedForward(ks, SwerveDriveConstants.calcKV, ka);

        angleServo = opMode.hardwareMap.get(CRServo.class, config.angleServoName);

        angleServo.setDirection(config.angleReverse);

        servoPotentiometer = opMode.hardwareMap.get(AnalogInput.class, config.absoluteEncoderName);

        angleOffset = config.offset;

        angleController.enableContinuousInput(-180, 180);

        angleController.setIZone(5);

        angleController.setTolerance(3);

        moduleNumber = config.moduleNumber;


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());


    }

    public static double round2dp(double number, int dp) {
        double temp = Math.pow(10, dp);
        double temp1 = Math.round(number * temp);
        return temp1 / temp;
    }

    public void createFeedForward(double ks, double kv, double ka) {
        driveFeedforward = new SimpleMotorFeedforward(
                ks,
                kv,
                ka);
    }

    public double getAngleKP() {
        return angleController.getP();
    }

    public void setAngleKP(double val) {
        angleController.setP(val);
    }

    public double getAngleKI() {
        return angleController.getI();
    }

    public void setAngleKI(double val) {
        angleController.setI(val);
    }

    public double getAngleKD() {
        return angleController.getD();
    }

    public void setAngleKD(double val) {
        angleController.setD(val);
    }

    public double getAnglePIDMAX() {
        return MAX_ANGLE_PID;
    }

    public void setAnglePIDMAX(double val) {
        MAX_ANGLE_PID=val;
    }

    public double getDriveKP() {
        return driveController.getP();
    }

    public void setDriveKP(double val) {
        driveController.setP(val);
    }

    public double getDriveKI() {
        return driveController.getI();
    }

    public void setDriveKI(double val) {
        driveController.setI(val);
    }

    public double getDriveKD() {
        return driveController.getD();
    }

    public void setDriveKD(double val) {
        driveController.setD(val);
    }

    public void setSpeedOpenLoop(SwerveModuleState state) {
        double power = state.speedMetersPerSecond / SwerveDriveConstants.maxSpeedMetersPerSec;
        double clampedPower = MathUtils.clamp(power, -powerLimit, powerLimit);
        driveMotor.setPower(clampedPower); //-1.0 to 1.0
    }

    public void setSpeedClosedLoop(SwerveModuleState state) {

        double acceleration = (state.speedMetersPerSecond - lastSpeed) / 0.020;

        acceleration = Math.min(Math.max(acceleration, -acceLimit), acceLimit);

        feedForwardVolts = driveFeedforward.calculate(
                state.speedMetersPerSecond, acceleration);

        double feedforward = feedForwardVolts / 12;//volts to +/-1

        lastSpeed = state.speedMetersPerSecond;

        pidOut = driveController.calculate(getWheelSpeedMPS(), state.speedMetersPerSecond);

        driveMotor.setPower(feedforward + pidOut);
    }

    public void setAngle(SwerveModuleState state) {


        setpoint = state.angle.getDegrees();

        angleController.setSetpoint(setpoint);

        pidout = angleController.calculate(wheelDegs, setpoint);

        if (pidout > MAX_ANGLE_PID) pidout = MAX_ANGLE_PID;
        if (pidout < -MAX_ANGLE_PID) pidout = -MAX_ANGLE_PID;

        anglePID = pidout;

        angleServo.setPower(pidout);
    }

    public void setAngleTrapezoid(SwerveModuleState state) {
        setpoint = state.angle.getDegrees();

        double pidout =
                thetapidController.calculate(
                        wheelDegs, setpoint);

        angleServo.setPower(pidout);
    }



        public SwerveModuleState getState() {
        double speedMetersPerSecond = getWheelSpeedMPS();
        double angleRadians = getWheelAngleRad();
        return new SwerveModuleState(speedMetersPerSecond, new Rotation2d(angleRadians));
    }

    public void setState(SwerveModuleState state) {
        wheelDegs = getWheelAngleDeg();
        if (state.angle.getDegrees() != origState.angle.getDegrees()) {
            newState = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(wheelDegs));
            origState = state;

        }

        if (openLoop)
            setSpeedOpenLoop(newState);
        else
            setSpeedClosedLoop(newState);


        setAngle(newState);


    }

    public void setOpenLoop(boolean val) {
        openLoop = val;
    }

    public double getTargetMPS() {
        return newState.speedMetersPerSecond;
    }

    public double getWheelAngleRad() {
        return Units.degreesToRadians(wheelDegs);
    }

    public double getWheelPosition() {
        return driveMotor.getCurrentPosition() / SwerveDriveConstants.TICKS_PER_METER;
    }

    double getWheelAngleDeg() {
        volts = servoPotentiometer.getVoltage();
        potAngle = round2dp(volts * 360 / 3.3, 2);

        return Math.round(Math.IEEEremainder((potAngle + angleOffset), 360));
    }

    public double getPotVolts() {
        return servoPotentiometer.getVoltage();
    }

    public double getWheelSpeedMPS() {
        return driveMotor.getVelocity() / SwerveDriveConstants.TICKS_PER_METER;
    }

    public double getDrivePower() {
        return driveMotor.getPower();
    }

    @Override
    public void periodic() {

        if (log) {
            KoalaLog.log("TargetDegreesOrig" + moduleNumber, origState.angle.getDegrees(), true);
            KoalaLog.log("TargetDegreesOpt" + moduleNumber, newState.angle.getDegrees(), true);
            KoalaLog.log("CurrentDegrees" + moduleNumber, wheelDegs, true);
            KoalaLog.log("Volts" + moduleNumber, volts, true);
            KoalaLog.log("PIDOut" + moduleNumber, anglePID, true);
            KoalaLog.log("POTAngle" + moduleNumber, potAngle, true);

//            KoalaLog.log("DrivePower" + moduleNumber, getDrivePower());
//            KoalaLog.log("DriveSpeed" + moduleNumber, getTargetMPS());
//            KoalaLog.log("DrivePosition" + moduleNumber, getWheelPosition());
//            KoalaLog.log("DriveTicks" + moduleNumber, driveMotor.getCurrentPosition());

        }

        if (showTelemetry) {
            telemetry.addData("TargetDegreesOrig" + moduleNumber, origState.angle.getDegrees());
            telemetry.addData("TargetDegreesOpt" + moduleNumber, newState.angle.getDegrees());
            telemetry.addData("CurrentDegrees" + moduleNumber, wheelDegs);
            telemetry.addData("Volts" + moduleNumber, volts);
            telemetry.addData("PIDOut" + moduleNumber, anglePID);
            telemetry.addData("POTAngle" + moduleNumber, potAngle);
            telemetry.update();
//            KoalaLog.log("DrivePower" + moduleNumber, getDrivePower());
//            KoalaLog.log("DriveSpeed" + moduleNumber, getTargetMPS());
//            KoalaLog.log("DrivePosition" + moduleNumber, getWheelPosition());
//            KoalaLog.log("DriveTicks" + moduleNumber, driveMotor.getCurrentPosition());

        }


        if (angleController.atSetpoint()) {
            angleServo.setPower(0);
        }
    }


}
