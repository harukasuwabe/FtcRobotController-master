package org.firstinspires.ftc.teamcode.Autonomous.Functions.roadRunner;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RoadRunner {

    public static void RedCloseRightProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive, DcMotor motorLinearSlideLeft, DcMotor motorLinearSlideRight, Servo door, Servo arm){

        arm.setPosition(-0.2);
        Pose2d startPose = new Pose2d(16,-64, Math.toRadians(270));
        sampleMecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence RedCloseRightProp = sampleMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(20,-64,Math.toRadians(270)))//where the prop located (+8)
                .lineToLinearHeading(new Pose2d(18.5,-40,Math.toRadians(200)))//where the prop located
                .lineToLinearHeading(new Pose2d(17,-60, Math.toRadians(0)))
                .build();
        TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(17,-60, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(56.85,-41.5, Math.toRadians(0)))//final position of the board
                .build();
        TrajectorySequence park = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(56.85,-42, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(54.85,-63, Math.toRadians(0)))
                .build();

        sampleMecanumDrive.followTrajectorySequence(RedCloseRightProp);
        Movement.linearSlides(1100, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        door.setPosition(-1);
        sampleMecanumDrive.followTrajectorySequence(slide);
        arm.setPosition(1);
        door.setPosition(1);
        try {
            sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Movement.linearSlides(0, telemetry, motorLinearSlideLeft, motorLinearSlideRight);

        try {
            sleep(600);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        sampleMecanumDrive.followTrajectorySequence(park);
    }



    public static void RedCloseCenterProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive, DcMotor motorLinearSlideLeft, DcMotor motorLinearSlideRight, Servo door, Servo arm){
        arm.setPosition(-0.2);
        Pose2d startPose = new Pose2d(17,-64, Math.toRadians(270));
        sampleMecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence RedCloseCenterProp = sampleMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(20,-64,Math.toRadians(270)))//where the prop located
                .lineToLinearHeading(new Pose2d(12,-44,Math.toRadians(270)))
                .build();

        TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(12,-32, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(56.85,-42, Math.toRadians(0)))
                .build();
        TrajectorySequence park = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(56.85,-42, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(54.85,-63, Math.toRadians(0)))
                        .build();

        sampleMecanumDrive.followTrajectorySequence(RedCloseCenterProp);
        Movement.linearSlides(1100, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        door.setPosition(-1);
        sampleMecanumDrive.followTrajectorySequence(slide);
        arm.setPosition(1);
        door.setPosition(1);
        try {
            sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Movement.linearSlides(0, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        try {
            sleep(600);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        sampleMecanumDrive.followTrajectorySequence(park);

    }
    public static void RedCloseLeftProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive, DcMotor motorLinearSlideLeft, DcMotor motorLinearSlideRight, Servo door, Servo arm){
        arm.setPosition(-0.2);
        Pose2d startPose = new Pose2d(17,-64, Math.toRadians(270)); //-8
        sampleMecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence RedCloseLeftProp = sampleMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(20,-64,Math.toRadians(270)))//where the prop located
                .lineToLinearHeading(new Pose2d(6,-35,Math.toRadians(0)))

                .build();

        TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(6,-40,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(56.85,-46, Math.toRadians(0)))
                .build();
        TrajectorySequence park = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(56.85,-48, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(54.85,-63, Math.toRadians(0)))
                .build();
        sampleMecanumDrive.followTrajectorySequence(RedCloseLeftProp);
        Movement.linearSlides(1100, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        door.setPosition(-1);
        sampleMecanumDrive.followTrajectorySequence(slide);
        arm.setPosition(1);
        door.setPosition(1);
        try {
            sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Movement.linearSlides(0, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        try {
            sleep(600);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        sampleMecanumDrive.followTrajectorySequence(park);
    }
    public static void RedFarRightProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive, DcMotor motorLinearSlideLeft, DcMotor motorLinearSlideRight, Servo door, Servo arm){
        Pose2d startPose = new Pose2d(-33,-64, Math.toRadians(270));//recheck the x starting position
        Pose2d close = new Pose2d(56.85,-60, Math.toRadians(0));
        Pose2d far = new Pose2d(56.85,-12, Math.toRadians(0));
        sampleMecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence RedFarRightProp = sampleMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-38,-64,Math.toRadians(270)))//where the prop located
                .lineToLinearHeading(new Pose2d(-22,-35,Math.toRadians(200)))
                .lineToLinearHeading(new Pose2d(-36,-60,Math.toRadians(180)))//change if needed
                .lineToLinearHeading(new Pose2d(56.85,-60, Math.toRadians(0)))

//                  This is for the far- replace this with the top two lines
//                .lineToLinearHeading(new Pose2d(-38,-64,Math.toRadians(270)))//where the prop located
//                .lineToLinearHeading(new Pose2d(-22,-35,Math.toRadians(200)))
//                .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(90))
//                .lineToLinearHeading(new Pose2d(54.85,-12, Math.toRadians(0)))

                .build();
        TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(close)
//      This is for the far- if far replace with the line above
//      TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(far)
                .lineToLinearHeading(new Pose2d(56.85,-41, Math.toRadians(0)))
                .build();

//        TrajectorySequence park = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(56.85,-41, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(56.85,-63, Math.toRadians(0)))
//                .build();
        sampleMecanumDrive.followTrajectorySequence(RedFarRightProp);
        Movement.linearSlides(1100, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        door.setPosition(-1);
        sampleMecanumDrive.followTrajectorySequence(slide);
        arm.setPosition(1);
        door.setPosition(1);
        try {
            sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Movement.linearSlides(0, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        try {
            sleep(600);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
//        sampleMecanumDrive.followTrajectorySequence(park);
    }

    public static void RedFarCenterProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive, DcMotor motorLinearSlideLeft, DcMotor motorLinearSlideRight, Servo door, Servo arm){
        Pose2d startPose = new Pose2d(-33,-64, Math.toRadians(270));
        Pose2d close = new Pose2d(56.85,-60, Math.toRadians(0));
        Pose2d far = new Pose2d(56.85,-12, Math.toRadians(0));
        sampleMecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence RedFarCenterProp = sampleMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-38,-64,Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-27,-32,Math.toRadians(270)))//where the prop located
                .lineToLinearHeading(new Pose2d(-37,-60,Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(56.85,-60, Math.toRadians(0)))
                //this is for the far
//                .lineToLinearHeading(new Pose2d(-38,-64,Math.toRadians(270)))
//                .lineToLinearHeading(new Pose2d(-27,-32,Math.toRadians(270)))//where the prop located
//                .lineToLinearHeading(new Pose2d(-40,-12,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(56.85,-12, Math.toRadians(0)))
                .build();
        TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(close)
//      This is for the far- if far replace with the line above
//      TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(far)
                .lineToLinearHeading(new Pose2d(56.85,-42, Math.toRadians(0)))
                .build();
//
//       if parking is needed
//        TrajectorySequence park = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(56.85,-42, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(56.85,-63, Math.toRadians(0)))
//                .build();

        sampleMecanumDrive.followTrajectorySequence(RedFarCenterProp);
        Movement.linearSlides(1100, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        door.setPosition(-1);
        sampleMecanumDrive.followTrajectorySequence(slide);
        arm.setPosition(1);
        door.setPosition(1);
        try {
            sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Movement.linearSlides(0, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        try {
            sleep(600);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
//        sampleMecanumDrive.followTrajectorySequence(park);
    }
    public static void RedFarLeftProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive, DcMotor motorLinearSlideLeft, DcMotor motorLinearSlideRight, Servo door, Servo arm){
        Pose2d startPose = new Pose2d(-36,-64, Math.toRadians(270));
        Pose2d close = new Pose2d(56.85,-60, Math.toRadians(270));
        Pose2d far = new Pose2d(56.85,-12, Math.toRadians(0));
        sampleMecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence RedFarLeftProp = sampleMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-38,-64,Math.toRadians(270)))//where the prop located
                .lineToLinearHeading(new Pose2d(-36,-38,Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-36,-60,Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(56.85,-60, Math.toRadians(270)))

                //This is going to be from the farther side
//                .lineToLinearHeading(new Pose2d(-38,-64,Math.toRadians(270)))//where the prop located
//                .lineToLinearHeading(new Pose2d(-36,-38,Math.toRadians(320)))
//                .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(56.85,-12, Math.toRadians(0)))
                .build();
        TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(close)
        //TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(far)
                .lineToLinearHeading(new Pose2d(56.85,-46, Math.toRadians(0)))
                .build();
//        TrajectorySequence park = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(56.85,-46, Math.toRadians(0)))
//                        .lineToLinearHeading(new Pose2d(56.85,-63, Math.toRadians(0)))
//                                .build();
        sampleMecanumDrive.followTrajectorySequence(RedFarLeftProp);
        Movement.linearSlides(1100, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        door.setPosition(-1);
        sampleMecanumDrive.followTrajectorySequence(slide);
        arm.setPosition(1);
        door.setPosition(1);
        try {
            sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Movement.linearSlides(0, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        try {
            sleep(600);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
//        sampleMecanumDrive.followTrajectorySequence(park);
    }
    public static void BlueCloseRightProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive, DcMotor motorLinearSlideLeft, DcMotor motorLinearSlideRight, Servo door, Servo arm){
        Pose2d startPose = new Pose2d(16,64, Math.toRadians(90));
        sampleMecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence BlueCloseRightProp = sampleMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(20,64,Math.toRadians(90)))//where the prop located (+8)
                .lineToLinearHeading(new Pose2d(18.5,40,Math.toRadians(120)))//where the prop located
                .lineToLinearHeading(new Pose2d(17,60, Math.toRadians(0)))
                .build();
        TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(17,60, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(56.85,41.5, Math.toRadians(0)))//final position of the board
                .build();
        TrajectorySequence park = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(56.85,42, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(54.85,63, Math.toRadians(0)))
                .build();
        sampleMecanumDrive.followTrajectorySequence(BlueCloseRightProp);
        Movement.linearSlides(1100, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        door.setPosition(-1);
        sampleMecanumDrive.followTrajectorySequence(slide);
        arm.setPosition(1);
        door.setPosition(1);
        try {
            sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Movement.linearSlides(0, telemetry, motorLinearSlideLeft, motorLinearSlideRight);

        try {
            sleep(600);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        sampleMecanumDrive.followTrajectorySequence(park);

    }
    public static void BlueCloseCenterProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive, DcMotor motorLinearSlideLeft, DcMotor motorLinearSlideRight, Servo door, Servo arm){
        Pose2d startPose = new Pose2d(17, 64, Math.toRadians(90));
        sampleMecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence BlueCloseCenterProp = sampleMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(20,64,Math.toRadians(90)))//where the prop located (+8)
                .lineToLinearHeading(new Pose2d(12,44,Math.toRadians(90)))
                .build();
        TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(12,44,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(56.85,52, Math.toRadians(0)))
                .build();
        TrajectorySequence park = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(56.85,52, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(54.85,63, Math.toRadians(0)))
                .build();
        sampleMecanumDrive.followTrajectorySequence(BlueCloseCenterProp);
        Movement.linearSlides(1100, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        door.setPosition(-1);
        sampleMecanumDrive.followTrajectorySequence(slide);
        arm.setPosition(1);
        door.setPosition(1);
        try {
            sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Movement.linearSlides(0, telemetry, motorLinearSlideLeft, motorLinearSlideRight);

        try {
            sleep(600);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        sampleMecanumDrive.followTrajectorySequence(park);
    }
    public static void BlueCloseLeftProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive, DcMotor motorLinearSlideLeft, DcMotor motorLinearSlideRight, Servo door, Servo arm){
        Pose2d startPose = new Pose2d(17, 64, Math.toRadians(90));
        sampleMecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence BlueCloseLeftProp = sampleMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(20,64,Math.toRadians(270)))//where the prop located (+8)
                .lineToLinearHeading(new Pose2d(6,35,Math.toRadians(0)))
                .build();

        TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(6,35,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(56.85,46, Math.toRadians(0)))
                .build();
        TrajectorySequence park = sampleMecanumDrive.trajectorySequenceBuilder(new Pose2d(56.85,-48, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(54.85,-63, Math.toRadians(0)))
                .build();
        sampleMecanumDrive.followTrajectorySequence(BlueCloseLeftProp);
        Movement.linearSlides(1100, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        door.setPosition(-1);
        sampleMecanumDrive.followTrajectorySequence(slide);
        arm.setPosition(1);
        door.setPosition(1);
        try {
            sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Movement.linearSlides(0, telemetry, motorLinearSlideLeft, motorLinearSlideRight);

        try {
            sleep(600);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        sampleMecanumDrive.followTrajectorySequence(park);
    }


    public static void BlueFarRightProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive, DcMotor motorLinearSlideLeft, DcMotor motorLinearSlideRight, Servo door, Servo arm){
        Pose2d startPose = new Pose2d(-38, 73, Math.toRadians(90));
        Pose2d close = new Pose2d(54.85,60, Math.toRadians(0));
        Pose2d far = new Pose2d(54.85, 12, Math.toRadians(0));
        sampleMecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence BlueFarRightProp = sampleMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-40,35,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-38,60,Math.toRadians(270)))//change if needed
                .lineToLinearHeading(new Pose2d(54.85,60, Math.toRadians(0)))
//                  This is for the far
//                .lineToLinearHeading(new Pose2d(-40,35,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(-37,12,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(54.85,12, Math.toRadians(0)))
                .build();
        TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(close)
//        TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(far)
                .lineToLinearHeading(new Pose2d(54.85,28, Math.toRadians(0)))
                .build();
        sampleMecanumDrive.followTrajectorySequence(BlueFarRightProp);
        Movement.linearSlides(1100, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        door.setPosition(-1);
        sampleMecanumDrive.followTrajectorySequence(slide);
        arm.setPosition(1);
        door.setPosition(1);
        try {
            sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Movement.linearSlides(0, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
    }
    public static void BlueFarCenterProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive, DcMotor motorLinearSlideLeft, DcMotor motorLinearSlideRight, Servo door, Servo arm){
        Pose2d startPose = new Pose2d(-38, 73, Math.toRadians(90));
        Pose2d close = new Pose2d(54.85, 60, Math.toRadians(0));
        Pose2d far = new Pose2d(54.85, 12, Math.toRadians(0));
        sampleMecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence BlueFarCenterProp = sampleMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-38,32,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-38,60,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(54.85,60, Math.toRadians(0)))

                //this is for the far
//                .lineToLinearHeading(new Pose2d(-38,32,Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(-56,43,Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(-40,12,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(54.85,12, Math.toRadians(0)))
                .build();
        TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(close)
//        TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(close)
                .lineToLinearHeading(new Pose2d(54.85,35, Math.toRadians(0)))
                .build();
        sampleMecanumDrive.followTrajectorySequence(BlueFarCenterProp);
        Movement.linearSlides(1100, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        door.setPosition(-1);
        sampleMecanumDrive.followTrajectorySequence(slide);
        arm.setPosition(1);
        door.setPosition(1);
        try {
            sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Movement.linearSlides(0, telemetry, motorLinearSlideLeft, motorLinearSlideRight);




    }
    public static void BlueFarLeftProp(Telemetry telemetry, SampleMecanumDrive sampleMecanumDrive, DcMotor motorLinearSlideLeft, DcMotor motorLinearSlideRight, Servo door, Servo arm){
        Pose2d startPose = new Pose2d(-38, 73, Math.toRadians(90));
        Pose2d close = new Pose2d(54.85, 60, Math.toRadians(0));
        Pose2d far = new Pose2d(54.85, 12, Math.toRadians(0));
        sampleMecanumDrive.setPoseEstimate(startPose);
        TrajectorySequence BlueFarLeftProp = sampleMecanumDrive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-33,35,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-38,60,Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(54.85,60, Math.toRadians(0)))
                //This is going to be from the farther side
//
//                .lineToLinearHeading(new Pose2d(-33,35,Math.toRadians(180)))
//                .lineToLinearHeading(new Pose2d(-37,12,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(54.85,12, Math.toRadians(0)))
                .build();
        TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(close)
//        TrajectorySequence slide = sampleMecanumDrive.trajectorySequenceBuilder(far)
                .lineToLinearHeading(new Pose2d(54.85,41, Math.toRadians(0)))
                        .build();

        sampleMecanumDrive.followTrajectorySequence(BlueFarLeftProp);
        Movement.linearSlides(1100, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
        door.setPosition(-1);
        sampleMecanumDrive.followTrajectorySequence(slide);
        arm.setPosition(1);
        door.setPosition(1);
        try {
            sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Movement.linearSlides(0, telemetry, motorLinearSlideLeft, motorLinearSlideRight);
    }



}
