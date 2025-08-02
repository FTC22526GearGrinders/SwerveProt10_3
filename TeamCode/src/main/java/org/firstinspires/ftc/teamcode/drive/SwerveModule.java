package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Units;

public class SwerveModule extends SubsystemBase {


    private final PIDFController driveController = new PIDFController(0.01, 0, 0, 0);
    private final CustomPIDFController angleController = new CustomPIDFController(.01, 0., 0., 0);
    private final boolean showTelemetry = true;
    public CRServo angleServo;
    public DcMotorEx driveMotor;
    public AnalogInput servoPotentiometer;
    public double angleOffset;
    public double angleSetPoint;
    public boolean openLoop;
    public int moduleNumber;
    public Telemetry telemetry;
    public double setpoint = 0;
    public double anglePID = 0;
    public double wheelDegs = 0;
    public double drivePower = 0;
    public double driveSpeedMetersPerSecond = 0;

    public SwerveModule(SwerveModuleConfig config, CommandOpMode opMode) {
        driveMotor = opMode.hardwareMap.get(DcMotorEx.class, config.driveMotorName);

        angleServo = opMode.hardwareMap.get(CRServo.class, config.angleServoName);
        angleServo.setDirection(config.angleReverse);

        servoPotentiometer = opMode.hardwareMap.get(AnalogInput.class, config.absoluteEncoderName);

        angleOffset = config.offset;


        //angleController = config.anglePIDFController;

        angleController.enableContinuousInput(-180, 180);
        angleController.setTolerance(3);


        moduleNumber = config.moduleNumber;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

    }

    public void setAngleKP(double val) {
        angleController.setP(val);
    }

    public double getAngleKP(){
        return angleController.getP();
    }

    public void setAngleKI(double val) {
        angleController.setI(val);
    }

    public double getAngleKI(){
        return angleController.getI();
    }


    public void setAngleKD(double val) {
        angleController.setD(val);
    }

    public double getAngleKD(){
        return angleController.getD();
    }


    public void setDriveKP(double val) {
        driveController.setP(val);
    }

    public double getDriveKP(){
        return driveController.getP();
    }

    public void setDriveKI(double val) {
        driveController.setI(val);
    }

    public double getDriveKI(){
        return driveController.getI();
    }


    public void setDriveKD(double val) {
        driveController.setD(val);
    }

    public double getDriveKD(){
        return driveController.getD();
    }



    public void setSpeedOpenLoop(SwerveModuleState state) {
        double speed = state.speedMetersPerSecond;
        driveSpeedMetersPerSecond = speed;
        double power = speed / SwerveDriveConstants.maxSpeedMetersPerSec;
        if (power > 1) power = 1;
        if (power < -1) power = -1;
        drivePower = power;
        driveMotor.setPower(power); //-1.0 to 1.0
    }

    public void setSpeedClosedLoop(SwerveModuleState state) {
        double speed = state.speedMetersPerSecond;
        driveSpeedMetersPerSecond = speed;
        double pidOut = driveController.calculate(getWheelSpeedMPS(), speed);

        driveMotor.setPower(pidOut);
    }

    public void setAngle(SwerveModuleState state) {
        angleSetPoint = state.angle.getDegrees();

        setpoint = angleSetPoint;
        angleController.setSetPoint(setpoint);

        wheelDegs = getWheelAngleDeg();
        double pidout = angleController.calculate(wheelDegs);

        if (pidout > 1) pidout = 1;
        if (pidout < -1) pidout = -1;

        anglePID = pidout;

        angleServo.setPower(pidout);
    }

    public SwerveModuleState getState() {
        double speedMetersPerSecond = getWheelSpeedMPS();
        double angleRadians = getWheelAngleRad();
        return new SwerveModuleState(speedMetersPerSecond, new Rotation2d(angleRadians));
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState newState = SwerveModuleState.optimize(state, new Rotation2d(getWheelAngleRad()));

        setSpeedOpenLoop(newState);

        setAngle(newState);

        if (openLoop)
            setSpeedOpenLoop(newState);
        else
            setSpeedClosedLoop(newState);
    }

    public void setOpenLoop(boolean val) {
        openLoop = val;
    }

    public double getWheelAngleRad() {
        return getWheelAngleDeg() * Math.PI / 180;
    }

    public double getWheelPosition() {
        return driveMotor.getCurrentPosition() / SwerveDriveConstants.TICKS_PER_METER;
    }

    double getWheelAngleDeg() {
        double volts = servoPotentiometer.getVoltage();
        double potAngle = volts * 360 / 3.3;
        return Math.IEEEremainder((potAngle + angleOffset), 360);
    }

    public double getWheelSpeedMPS() {
        double motorTicksPerSecond = driveMotor.getVelocity();
        double wheelRevolutionsPerSecond = motorTicksPerSecond / SwerveDriveConstants.TICKS_PER_REVOLUTION;
        return Units.inchesToMeters(wheelRevolutionsPerSecond * SwerveDriveConstants.WHEEL_CIRCUMFERENCE_INCHES);
    }

    @Override
    public void periodic() {
        if (showTelemetry) {
            telemetry.addData("CurrentDegrees" + moduleNumber, getWheelAngleDeg());
//        telemetry.addData("SetPoint" + moduleNumber, setpoint);
//        telemetry.addData("PIDOut" + moduleNumber, anglePID);
//        telemetry.addData("DrivePower" + moduleNumber, drivePower);
            telemetry.addData("DriveSpeed" + moduleNumber, driveSpeedMetersPerSecond);
            telemetry.addData("DrivePosition" + moduleNumber, getWheelPosition());
            telemetry.addData("DriveTicks" + moduleNumber, driveMotor.getCurrentPosition());

        }
        if (angleController.atSetPoint()) {
            angleServo.setPower(0);
        }
    }
}
