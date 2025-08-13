package org.firstinspires.ftc.teamcode.simulation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.drive.SwerveDriveConstants;
import org.firstinspires.ftc.teamcode.utils.HolonomicDriveController;

import java.util.concurrent.TimeUnit;


public class RunTrajectoryCommandSim extends CommandBase {

    private final CommandOpMode myOpmode;
    private final SwerveDriveSim driveSim;
    private final PIDController xpidController = new PIDController(.1, 0, 0);
    private final PIDController ypidController = new PIDController(.1, 0, 0);
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            5, 8);
    private final ProfiledPIDController thetapidController = new ProfiledPIDController(2, 0, 0, constraints);

    public HolonomicDriveController m_holonomicController
            = new HolonomicDriveController(
            xpidController,
            ypidController,
            thetapidController);

    private Trajectory traj;
    private Timing.Timer timer;
    private double curTime;
    private double trajLength;

    public RunTrajectoryCommandSim(SwerveDriveSim driveSim, Trajectory traj, CommandOpMode opMode) {
        this.driveSim = driveSim;
        this.traj = traj;
        myOpmode = opMode;
        addRequirements(this.driveSim);
    }

    public static double round2dp(double number, int dp) {
        double temp = Math.pow(10, dp);
        double temp1 = Math.round(number * temp);
        return temp1 / temp;
    }

    @Override
    public void initialize() {
        Pose2d initialPose = traj.getInitialPose();
        driveSim.resetPose(initialPose);
        trajLength = traj.getTotalTimeSeconds();
        timer = new Timing.Timer((long) trajLength, TimeUnit.MILLISECONDS);
        timer.start();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        myOpmode.telemetry = new MultipleTelemetry(myOpmode.telemetry, dashboard.getTelemetry());
       // myOpmode.telemetry.addData("TRAJTIME", traj.getTotalTimeSeconds());
    }

    @Override
    public void execute() {

        curTime = timer.elapsedTime() / 1000.;
      //  myOpmode.telemetry.addData("CURTIME", curTime);
        Trajectory.State desiredState = traj.sample(curTime);
//        myOpmode.telemetry.addData("CURSTATE_X", desiredState.poseMeters.getX());
//        myOpmode.telemetry.addData("CURSTATE_Y", desiredState.poseMeters.getY());
//        myOpmode.telemetry.addData("CURSTATE_THETA", desiredState.poseMeters.getHeading());
//
//        myOpmode.telemetry.addData("CURSTATE_IMU", driveSim.getHeading().getDegrees());


        Rotation2d desiredRotation = desiredState.poseMeters.getRotation();


        ChassisSpeeds targetChassisSpeeds =
                m_holonomicController.calculate(driveSim.getPose(), desiredState, desiredRotation);
        SwerveModuleState[] targetModuleStates = SwerveDriveConstants.swerveKinematics.toSwerveModuleStates(targetChassisSpeeds);

        for (SwerveModuleSim module : driveSim.modules) {
            module.setState(targetModuleStates[module.moduleNumber]);
        }


       // myOpmode.telemetry.update();

    }

    @Override
    public void end(boolean interrupted) {
        driveSim.drive(0, 0, 0, true);
        myOpmode.telemetry.addData("ENDED", 0);
        myOpmode.telemetry.update();

    }

    @Override
    public boolean isFinished() {
        return curTime > 0 && trajLength > 0 && curTime > trajLength;
    }

}