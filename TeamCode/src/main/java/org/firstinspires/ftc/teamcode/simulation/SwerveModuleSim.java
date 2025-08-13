package org.firstinspires.ftc.teamcode.simulation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SwerveModuleConfig;
import org.firstinspires.ftc.teamcode.utils.Units;


public class SwerveModuleSim extends SubsystemBase {

    private final boolean showTelemetry = true;


    public double angleSetPoint;
    public boolean openLoop;
    public int moduleNumber;
    public Telemetry telemetry;
    public double setpoint = 0;
    public double feedForwardVolts;
    SwerveModuleState newState = new SwerveModuleState();
    private double driveWheelPosition;

    public SwerveModuleSim(SwerveModuleConfig config, CommandOpMode opMode) {

        moduleNumber = config.moduleNumber;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

    }


    public void setAngle(SwerveModuleState state) {
        angleSetPoint = state.angle.getDegrees();
        setpoint = angleSetPoint;

    }

    public SwerveModuleState getState() {
        double speedMetersPerSecond = getWheelSpeedMPS();
        double angleRadians = getWheelAngleRad();
        return new SwerveModuleState(speedMetersPerSecond, new Rotation2d(angleRadians));
    }

    public void setState(SwerveModuleState state) {
        newState = SwerveModuleState.optimize(state, new Rotation2d(getWheelAngleRad()));

        setAngle(newState);

        setDrivePosition(newState);

    }

    public double getDrivePosition() {
        return driveWheelPosition;
    }

    public void setDrivePosition(SwerveModuleState state) {
        driveWheelPosition += state.speedMetersPerSecond * .02;
    }

    public double getTargetIPS() {
        return newState.speedMetersPerSecond;
    }

    public double getWheelAngleRad() {
        return newState.angle.getRadians();
    }

    public double getWheelAngleDeg() {
        return Units.radiansToDegrees(newState.angle.getRadians());
    }

    public double getWheelSpeedMPS() {
        return newState.speedMetersPerSecond;
    }

    @Override
    public void periodic() {
        if (showTelemetry) {
            telemetry.addData("CurrentDegrees" + moduleNumber, getWheelAngleDeg());
        telemetry.addData("SetPoint" + moduleNumber, setpoint);
    //    telemetry.addData("PIDOut" + moduleNumber, anglePID);

            telemetry.addData("DriveSpeed" + moduleNumber, getTargetIPS());
            telemetry.addData("DrivePosition" + moduleNumber, getDrivePosition());

            telemetry.update();
        }


    }


}
