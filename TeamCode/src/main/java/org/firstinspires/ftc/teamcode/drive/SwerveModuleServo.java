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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.RollingAverage;
import org.firstinspires.ftc.teamcode.utils.Units;


public class SwerveModuleServo extends SubsystemBase {

    private final PIDControllerFRC driveController = new PIDControllerFRC(0.01, 0, 0, .02);
    private final PIDControllerFRC angleController = new PIDControllerFRC(.015, 0.0, 0., 0.02);
    private final double acceLimit = 5;
    private final double powerLimit = 1;
    private final double ks = 0;
    private final double ka = 0;
    public TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            200, 200);
    public final ProfiledPIDController thetapidController = new ProfiledPIDController(.01, 0, 0, constraints);
    public int showTelemetry = 0;
    public Servo angleServo;
    public DcMotorEx driveMotor;
    public AnalogInput servoPotentiometer;
    public double angleOffset;
    public boolean openLoop;
    public int moduleNumber;
    public Telemetry telemetry;
    public double setpoint = 0;
    public double lastSetpoint = 1;
    public double wheelDegs = 0;
    public double pidout;
    public double feedForwardVolts;
    public SwerveModuleState newState = new SwerveModuleState();
    public SwerveModuleState origState = new SwerveModuleState();
    public double sampleCount;
    public double MAX_ANGLE_PID = .2;
    public double lastPotSpeed;
    public double lastAngle;
    public double lastPotReadTime;
    public double angleChange;
    public double timeChange;

    public TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
    public TrapezoidProfile.State currentState = new TrapezoidProfile.State(0, 0);
    public TrapezoidProfile.State previousState = new TrapezoidProfile.State(currentState.position, currentState.velocity);


    public ElapsedTime potTimer = new ElapsedTime();

    public double minServoPosition = .02;
    public double maxServoPosition = .98;
    public double midServoPosition = .5;
    public double servoPositionRange = maxServoPosition - minServoPosition;//.96
    public double[] voltsAtMin = {.644, .637, .640, .640};
    public double[] voltsAtMax = {2.666, 2.65, 2.64, 2.66};
    public double[] voltsAtMid = {1.655, 1.643, 1.6, 1.6};

    public double degreesPerVolt = 360. / 3.3;
    public double minAngleDegrees = -110;//68
    public double maxAngleDegrees = 110;//288
    public double midAngleDegrees = 0;//288

    public double degreeRange = maxAngleDegrees - minAngleDegrees;//220

    public double posRangeDegRangeRatio = servoPositionRange / degreeRange;

    public double servoCmd;
    public double volts;
    TrapezoidProfile prof = new TrapezoidProfile(constraints, goal, currentState);
    private double Kvff = .1;
    private SimpleMotorFeedforward driveFeedforward;
    private double lastSpeed;
    private double pidOut;
    private double potAngle;
    private RollingAverage ra;
    //from logs and excel pot volts = 2.109 * servo position + .6102
    //servo position = (pot volts -.6102) / 2.109

    int tst;

    public SwerveModuleServo(SwerveModuleConfig config, CommandOpMode opMode) {
        driveMotor = opMode.hardwareMap.get(DcMotorEx.class, config.driveMotorName);

        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        createFeedForward(ks, SwerveDriveConstants.calcKV, ka);

        angleServo = opMode.hardwareMap.get(Servo.class, config.angleServoName);

        angleServo.setDirection(Servo.Direction.REVERSE);

        if(angleServo.getDirection()== Servo.Direction.FORWARD){
            double[] temp = voltsAtMin;
            voltsAtMin=voltsAtMax;
            voltsAtMax=temp;
            double temp1 = minAngleDegrees;
            minAngleDegrees=maxAngleDegrees;
            maxAngleDegrees=temp1;

        }


        servoPotentiometer = opMode.hardwareMap.get(AnalogInput.class, config.absoluteEncoderName);

        angleOffset = config.offset;

        angleController.enableContinuousInput(-180, 180);

        angleController.setIZone(5);

        angleController.setTolerance(3);

        moduleNumber = config.moduleNumber;

        ra = new RollingAverage(5);

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

    public void setThetaAngleKP(double val) {
        thetapidController.setP(val);
    }

    public void setThetaAngleKI(double val) {
        thetapidController.setI(val);
    }

    public void setThetaAngleKD(double val) {
        thetapidController.setD(val);
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
        MAX_ANGLE_PID = val;
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
        driveMotor.setPower(clampedPower);
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

    public double getServoPositionFromDegrees(double deg) {
        return midServoPosition + deg * posRangeDegRangeRatio;
    }

    public double getDegreesFromServoPosition() {
        return (getServoPosition() - midServoPosition) / posRangeDegRangeRatio;
    }

    public void setAngle(SwerveModuleState state) {
        double deg = state.angle.getDegrees();
        servoCmd = getServoPositionFromDegrees(deg);
        angleServo.setPosition(servoCmd);
    }

    public void setAngleDegrees(double deg) {
        double servoCmd = getServoPositionFromDegrees(deg);
        angleServo.setPosition(servoCmd);
    }


    public double getServoPosition() {
        return round2dp(angleServo.getPosition(), 2);
    }

    public void setServoPosition(double val) {
        angleServo.setPosition(val);
    }


    public void setAngleTrapezoid(SwerveModuleState state) {

        setpoint = state.angle.getDegrees();

        if (setpoint != lastSetpoint) {
            lastSetpoint = setpoint;
            goal.position = setpoint;
            thetapidController.reset(wheelDegs);
        }
        double verr = thetapidController.getVelocityError();

        double perr = thetapidController.getPositionError();

//        telemetry.addData("VERR", verr);
//
//        telemetry.addData("PERR", perr);

        pidout = thetapidController.calculate(
                wheelDegs, goal.position);

        // angleServo.setPower(pidout);
        //set PID to 0 when finding zero
    }


    public SwerveModuleState getState() {
        double speedMetersPerSecond = getWheelSpeedMPS();
        double angleRadians = getWheelAngleRad();
        return new SwerveModuleState(speedMetersPerSecond, new Rotation2d(angleRadians));
    }

    public void setState(SwerveModuleState state) {
        wheelDegs = getWheelAngleDeg();
        //check for new angle
        if (state.angle.getDegrees() != origState.angle.getDegrees()) {
            newState = state;
            newState = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(wheelDegs));
            //flip angle if it went out of range
            if (newState.angle.getDegrees() > maxAngleDegrees || newState.angle.getDegrees() < minAngleDegrees)
                newState = new SwerveModuleState(
                        -newState.speedMetersPerSecond,
                        newState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
            origState = state;
        }
        if (openLoop)
            setSpeedOpenLoop(newState);
        else
            setSpeedClosedLoop(newState);
//
//        setAngleTrapezoid(newState);
        setAngle(newState);

    }

    public double getSpeedFromPot() {
        volts = servoPotentiometer.getVoltage();
        potAngle = round2dp(volts * degreesPerVolt, 2);
        double readTime = potTimer.milliseconds();
        angleChange = potAngle - lastAngle;
        timeChange = lastPotReadTime - readTime;
        lastPotReadTime = readTime;
        // if angle crosses zero between reads then add 360 to compensagte
        if (lastAngle > potAngle) angleChange += 360;
        lastAngle = potAngle;
        return angleChange / timeChange;

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

    public double getWheelAngleDeg() {
        //-140 deg = .6 volts
        //140 deg = 2.7 volts
        //0 deg = 1.65 volts

        double diff = getPotVolts() - voltsAtMid[moduleNumber];
        potAngle = midAngleDegrees + (diff * degreesPerVolt);
        return potAngle;
    }

    public double getPotVolts() {
        //return ra.getAverage();
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
        ra.add(getPotVolts());

    }


}
