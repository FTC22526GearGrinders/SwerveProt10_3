package org.firstinspires.ftc.teamcode.utils;


import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SwerveDriveConstants;

public final class FieldConstantsSelect {

    //basket
    public double pickUpArmEX = 10.5;//11;
    public double driveTo = 10;

    public double basketPreY = 6;
    public double sampleDropOffZoneFromWall = 8;
    public double specStartX = -9;
    public double specDiffX = 6;
    public double specimenApproach = 6;
    public double tileToothDepth = .75;
    public double sampleLength = 3.5;
    public double sampleWidth = 1.5;

    public Pose2d basketSideStartPose;
    public Pose2d basketSideStrafePose;

    public Pose2d basketDeliverPose;
    public Pose2d innerYellowPickupPose;
    public Pose2d innerYellowPrePickupPose;
    public Pose2d midYellowPickupPose;
    public Pose2d midYellowPrePickupPose;
    public Pose2d outerYellowPrePose;
    public Pose2d outerYellowPickupPose;
    public Pose2d ascentZoneParkPose;
    public Pose2d ascentZonePickupPose;
    public double specimenDropAngle;
    //specimen
    public double specimenPickupAngle;
    public Pose2d specimenSideStartPose;
    public Pose2d sample1ObservationZoneDropPose;
    public Pose2d sample1ObservationZonePickupPose;
    public Pose2d sample2ObservationZoneDropPose;
    public Pose2d sample2ObservationZonePickupPose;
    public Pose2d sample3ObservationZoneDropPose;
    public Pose2d sample3ObservationZonePickupPose;

    public Pose2d firstStagePushInnerPose;
    public Vector2d secondStagePushInnerVector;
    public Vector2d thirdStagePushInnerVector;
    public Pose2d firstStagePushMidPose;
    public Vector2d secondStagePushMidVector;
    public Vector2d thirdStagePushMidVector;
    public Pose2d specimenDeliverPose1;
    public Pose2d specimenDeliverPose2;
    public Pose2d specimenDeliverPose3;
    public Pose2d specimenDeliverPose4;
    public Pose2d specimenDeliverApproachPose1;
    public Pose2d specimenDeliverApproachPose2;
    public Pose2d specimenDeliverApproachPose3;
    public Pose2d specimenDeliverApproachPose4;
    public Pose2d specimenPickupPose;
    public Pose2d specimenPickupApproachPose;
    public Pose2d specimenParkPose;
    public double outerYellowPickupAngle;

    public FieldConstantsSelect() {

        setBlue();
    }

    public void setBlue() {

        basketSideStartPose = new Pose2d(36, 72 - SwerveDriveConstants.robotWidth / 2., new Rotation2d(Math.toRadians(180)));
        basketSideStrafePose = new Pose2d(41, 66 - SwerveDriveConstants.robotWidth / 2., new Rotation2d(Math.toRadians(180)));
        basketDeliverPose = new Pose2d(57, 57,new Rotation2d( Units.degreesToRadians(-135)));


        innerYellowPrePickupPose = new Pose2d(48 + tileToothDepth / 2. + sampleWidth / 2.,
                24 + tileToothDepth / 2. + sampleLength / 2. + basketPreY + SwerveDriveConstants.robotLength / 2. + pickUpArmEX, new Rotation2d(Math.toRadians(-100)));

        innerYellowPickupPose = new Pose2d(48 + tileToothDepth / 2. + sampleWidth / 2.,
                24 + tileToothDepth / 2. + sampleLength / 2. + SwerveDriveConstants.robotLength / 2. + pickUpArmEX, new Rotation2d(Math.toRadians(-90)));

        midYellowPrePickupPose = new Pose2d(58 + tileToothDepth / 2. + sampleWidth / 2.,
                24 + tileToothDepth / 2. + sampleLength / 2. + basketPreY + SwerveDriveConstants.robotLength / 2. + pickUpArmEX, new Rotation2d(Math.toRadians(-100)));

        midYellowPickupPose = new Pose2d(58 + tileToothDepth / 2. + sampleWidth / 2.,
                24 + tileToothDepth / 2. + sampleLength / 2. + SwerveDriveConstants.robotLength / 2. + pickUpArmEX, new Rotation2d(Math.toRadians(-90)));

        outerYellowPrePose = new Pose2d(SwerveDriveConstants.fieldWidth / 2. - 1 - SwerveDriveConstants.robotLength / 2. - pickUpArmEX - driveTo, 25.5, new Rotation2d(Math.toRadians(-10)));
        outerYellowPickupPose = new Pose2d(SwerveDriveConstants.fieldWidth / 2. - 1 - SwerveDriveConstants.robotLength / 2. - pickUpArmEX, 25.5, new Rotation2d(Math.toRadians(0)));

        ascentZoneParkPose = new Pose2d(24, -12, new Rotation2d(Math.toRadians(180)));
        ascentZonePickupPose = new Pose2d(24, -12, new Rotation2d(Math.toRadians(180)));

        //specimen
        specimenDropAngle = Math.toRadians(90);
        specimenPickupAngle = Math.toRadians(-90);
        outerYellowPickupAngle = Math.toRadians(0);

        specimenSideStartPose = new Pose2d(specStartX, SwerveDriveConstants.fieldLength / 2. - SwerveDriveConstants.robotLength / 2., new Rotation2d(Math.toRadians(specimenDropAngle)));

        sample1ObservationZoneDropPose = new Pose2d(-48, SwerveDriveConstants.fieldLength / 2. - SwerveDriveConstants.robotLength / 2. - sampleDropOffZoneFromWall, new Rotation2d(Math.toRadians(180)));
        sample1ObservationZonePickupPose = new Pose2d(-48, SwerveDriveConstants.fieldLength / 2. - SwerveDriveConstants.robotLength / 2. - sampleDropOffZoneFromWall, new Rotation2d(Math.toRadians(180)));

        sample2ObservationZoneDropPose = new Pose2d(-58, SwerveDriveConstants.fieldLength / 2. - SwerveDriveConstants.robotLength / 2., new Rotation2d(Math.toRadians(specimenPickupAngle)));
        sample2ObservationZonePickupPose = new Pose2d(-58, SwerveDriveConstants.fieldLength / 2. - SwerveDriveConstants.robotLength / 2., new Rotation2d(Math.toRadians(specimenPickupAngle)));

        sample3ObservationZoneDropPose = new Pose2d(-48, SwerveDriveConstants.fieldLength / 2. - SwerveDriveConstants.robotLength / 2. - sampleDropOffZoneFromWall, new Rotation2d(Math.toRadians(specimenPickupAngle)));
        sample3ObservationZonePickupPose = new Pose2d(-48, SwerveDriveConstants.fieldLength / 2. - SwerveDriveConstants.robotLength / 2., new Rotation2d(Math.toRadians(specimenPickupAngle)));


        firstStagePushInnerPose = new Pose2d(-36, 42, new Rotation2d(Math.toRadians(180)));
        secondStagePushInnerVector = new Vector2d(-36, 10);
        thirdStagePushInnerVector = new Vector2d(-48, 10);

        firstStagePushMidPose = new Pose2d(-28, 42, new Rotation2d(Math.toRadians(180)));
        secondStagePushMidVector = new Vector2d(-40, 10);//
        thirdStagePushMidVector = new Vector2d(-58, 10);


        specimenDeliverPose1 = new Pose2d(specStartX, 24.5 + SwerveDriveConstants.robotLength / 2., new Rotation2d(Math.toRadians(specimenDropAngle)));
        specimenDeliverPose2 = new Pose2d(specStartX + specDiffX, 24 + SwerveDriveConstants.robotLength / 2., new Rotation2d(Math.toRadians(specimenDropAngle)));
        specimenDeliverPose3 = new Pose2d(specStartX + 2 * specDiffX, 24 + SwerveDriveConstants.robotLength / 2., new Rotation2d(Math.toRadians(specimenDropAngle)));
        specimenDeliverPose4 = new Pose2d(specStartX + 3 * specDiffX, 24 + SwerveDriveConstants.robotLength / 2., new Rotation2d(Math.toRadians(specimenDropAngle)));

        specimenDeliverApproachPose1 = new Pose2d(specStartX, 24 + specimenApproach + SwerveDriveConstants.robotLength / 2., new Rotation2d(Math.toRadians(specimenDropAngle)));
        specimenDeliverApproachPose2 = new Pose2d(specStartX + specDiffX, 25 + specimenApproach + SwerveDriveConstants.robotLength / 2., new Rotation2d(Math.toRadians(specimenDropAngle)));
        specimenDeliverApproachPose3 = new Pose2d(specStartX + 2 * specDiffX, 25 + specimenApproach + SwerveDriveConstants.robotLength / 2., new Rotation2d(Math.toRadians(specimenDropAngle)));
        specimenDeliverApproachPose4 = new Pose2d(specStartX + 3 * specDiffX, 25 + specimenApproach + SwerveDriveConstants.robotLength / 2., new Rotation2d(Math.toRadians(specimenDropAngle)));

        specimenPickupPose = new Pose2d(-36, SwerveDriveConstants.fieldLength / 2. - SwerveDriveConstants.robotLength / 2., new Rotation2d(Math.toRadians(specimenPickupAngle)));
        specimenPickupApproachPose = new Pose2d(-36, specimenPickupPose.getY() - 7, new Rotation2d(Math.toRadians(specimenPickupAngle)));

        specimenParkPose = new Pose2d(-48, 60 - SwerveDriveConstants.robotLength / 2. + 8, new Rotation2d(Math.toRadians(180)));
    }

    public void setRed() {
        basketSideStrafePose = flipBlueToRedPose(basketSideStrafePose);
        basketSideStartPose = flipBlueToRedPose(basketSideStartPose);
        basketDeliverPose = flipBlueToRedPose(basketDeliverPose);


        innerYellowPrePickupPose = flipBlueToRedPose(innerYellowPrePickupPose);

        midYellowPrePickupPose = flipBlueToRedPose(midYellowPrePickupPose);

        innerYellowPickupPose = flipBlueToRedPose(innerYellowPickupPose);

        midYellowPickupPose = flipBlueToRedPose(midYellowPickupPose);


        outerYellowPrePose = flipBlueToRedPose(outerYellowPrePose);
        outerYellowPickupPose = flipBlueToRedPose(outerYellowPickupPose);

        ascentZoneParkPose = flipBlueToRedPose(ascentZoneParkPose);
        ascentZonePickupPose = flipBlueToRedPose(ascentZonePickupPose);

        //specimen
        specimenDropAngle = Math.toRadians(-90);
        specimenPickupAngle = Math.toRadians(90);
        outerYellowPickupAngle = Math.toRadians(180);


        specimenSideStartPose = flipBlueToRedPose(specimenSideStartPose);

        sample1ObservationZoneDropPose = flipBlueToRedPose(sample1ObservationZoneDropPose);
        sample1ObservationZonePickupPose = flipBlueToRedPose(sample1ObservationZonePickupPose);
        sample2ObservationZoneDropPose = flipBlueToRedPose(sample2ObservationZoneDropPose);
        sample2ObservationZonePickupPose = flipBlueToRedPose(sample2ObservationZonePickupPose);
        sample3ObservationZoneDropPose = flipBlueToRedPose(sample3ObservationZoneDropPose);
        sample3ObservationZonePickupPose = flipBlueToRedPose(sample3ObservationZonePickupPose);


        firstStagePushInnerPose = flipBlueToRedPose(firstStagePushInnerPose);
        secondStagePushInnerVector = flipBlueToRedVector(secondStagePushInnerVector);
        thirdStagePushInnerVector = flipBlueToRedVector(thirdStagePushInnerVector);

        firstStagePushMidPose = flipBlueToRedPose(firstStagePushMidPose);
        secondStagePushMidVector = flipBlueToRedVector(secondStagePushMidVector);
        thirdStagePushMidVector = flipBlueToRedVector(thirdStagePushMidVector);


        specimenDeliverPose1 = flipBlueToRedPose(specimenDeliverPose1);
        specimenDeliverPose2 = flipBlueToRedPose(specimenDeliverPose2);
        specimenDeliverPose3 = flipBlueToRedPose(specimenDeliverPose3);
        specimenDeliverPose4 = flipBlueToRedPose(specimenDeliverPose4);

        specimenDeliverApproachPose1 = flipBlueToRedPose(specimenDeliverApproachPose1);
        specimenDeliverApproachPose2 = flipBlueToRedPose(specimenDeliverApproachPose2);
        specimenDeliverApproachPose3 = flipBlueToRedPose(specimenDeliverApproachPose3);
        specimenDeliverApproachPose4 = flipBlueToRedPose(specimenDeliverApproachPose4);

        specimenPickupPose = flipBlueToRedPose(specimenPickupPose);
        specimenPickupApproachPose = flipBlueToRedPose(specimenPickupApproachPose);
        specimenParkPose = flipBlueToRedPose(specimenParkPose);
    }

    Vector2d flipBlueToRedVector(Vector2d blue) {
        return new Vector2d(-blue.getX(), -blue.getY());
    }

    double flipBlueToRedHeading(Pose2d blue) {
        return blue.getHeading() >= 0 ? blue.getHeading() - Math.PI : blue.getHeading() + Math.PI;
    }

    Pose2d flipBlueToRedPose(Pose2d blue) {
        double heading = blue.getHeading() >= 0 ? blue.getHeading() - Math.PI : blue.getHeading()+ Math.PI;
        return new Pose2d(-blue.getX(), -blue.getY(), new Rotation2d(heading));
    }
}



