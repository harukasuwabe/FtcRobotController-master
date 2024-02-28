package org.firstinspires.ftc.teamcode.Autonomous.Functions.roadRunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RoadRunner {

    public static void RedCloseRightProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(17,-73, Math.toRadians(270));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory RedCloseRightProp = sampleMecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(13,-31,Math.toRadians(180)))
//                .forward(9.5)
//                .strafeRight(24)
//                .turn(Math.toRadians(180))
//                .forward(36)
                .build();
        sampleMecanumDrive.followTrajectory(RedCloseRightProp);
    }
    public static void RedCloseCenterProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(17,-73, Math.toRadians(270));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory RedCloseCenterProp = sampleMecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12,-28,Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(54.85,-35, Math.toRadians(0)))
//                .forward(9.5)
//                .strafeRight(24)
//                .turn(Math.toRadians(180))
//                .forward(36)
                .build();
        sampleMecanumDrive.followTrajectory(RedCloseCenterProp);
    }
    public static void RedCloseLeftProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(17,-73, Math.toRadians(270));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory RedCloseLeftProp = sampleMecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(6,-30,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(54.85,-43, Math.toRadians(0)))
//                .forward(9.5)
//                .strafeRight(24)
//                .turn(Math.toRadians(180))
//                .forward(36)
                .build();
        sampleMecanumDrive.followTrajectory(RedCloseLeftProp);
    }
    public static void RedFarRightProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(-50,-73, Math.toRadians(270));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory RedFarRightProp = sampleMecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35,-30,Math.toRadians(180)))
//                .forward(9.5)
//                .strafeRight(24)
//                .turn(Math.toRadians(180))
//                .forward(36)
                .build();
        sampleMecanumDrive.followTrajectory(RedFarRightProp);
    }

    public static void RedFarCenterProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(-50,-73, Math.toRadians(270));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory RedFarCenterProp = sampleMecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-42,-28,Math.toRadians(270)))
//                .forward(9.5)
//                .strafeRight(24)
//                .turn(Math.toRadians(180))
//                .forward(36)
                .build();
        sampleMecanumDrive.followTrajectory(RedFarCenterProp);
    }
    public static void RedFarLeftProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(-50,-73, Math.toRadians(270));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory RedFarLeftProp = sampleMecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-43,-30,Math.toRadians(0)))
//                .forward(9.5)
//                .strafeRight(24)
//                .turn(Math.toRadians(180))
//                .forward(36)
                .build();
        sampleMecanumDrive.followTrajectory(RedFarLeftProp);
    }
    public static void BlueCloseRightProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(17, 73, Math.toRadians(90));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory BlueCloseRightProp = sampleMecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(6,30,Math.toRadians(0)))
//                .forward(9.5)
//                .strafeRight(24)
//                .turn(Math.toRadians(180))
//                .forward(36)
                .build();
        sampleMecanumDrive.followTrajectory(BlueCloseRightProp);
    }
    public static void BlueCloseCenterProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(17, 73, Math.toRadians(90));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory BlueCloseCenterProp = sampleMecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12,28,Math.toRadians(90)))
//                .forward(9.5)
//                .strafeRight(24)
//                .turn(Math.toRadians(180))
//                .forward(36)
                .build();
        sampleMecanumDrive.followTrajectory(BlueCloseCenterProp);
    }
    public static void BlueCloseLeftProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(17, 73, Math.toRadians(90));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory BlueCloseLeftProp = sampleMecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(13,31,Math.toRadians(180)))
//                .forward(9.5)
//                .strafeRight(24)
//                .turn(Math.toRadians(180))
//                .forward(36)
                .build();
        sampleMecanumDrive.followTrajectory(BlueCloseLeftProp);
    }


    public static void BlueFarRightProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(-50, 73, Math.toRadians(90));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory BlueFarRightProp = sampleMecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-43,30,Math.toRadians(0)))
//                .forward(9.5)
//                .strafeRight(24)
//                .turn(Math.toRadians(180))
//                .forward(36)
                .build();
        sampleMecanumDrive.followTrajectory(BlueFarRightProp);
    }
    public static void BlueFarCenterProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(-50, 73, Math.toRadians(90));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory BlueFarCenterProp = sampleMecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-42,28,Math.toRadians(90)))
//                .forward(9.5)
//                .strafeRight(24)
//                .turn(Math.toRadians(180))
//                .forward(36)
                .build();
        sampleMecanumDrive.followTrajectory(BlueFarCenterProp);
    }
    public static void BlueFarLeftProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive){
        Pose2d startPose = new Pose2d(-50, 73, Math.toRadians(90));
        sampleMecanumDrive.setPoseEstimate(startPose);
        Trajectory BlueFarLeftProp = sampleMecanumDrive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35,30,Math.toRadians(180)))
//                .forward(9.5)
//                .strafeRight(24)
//                .turn(Math.toRadians(180))
//                .forward(36)
                .build();
        sampleMecanumDrive.followTrajectory(BlueFarLeftProp);
    }



}
