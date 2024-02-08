package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;


@TeleOp(name = "NewTeleop")
public class NewTeleop extends LinearOpMode {
    private DcMotor back_left;
    private DcMotor front_right;
    private DcMotor front_left;
    private DcMotor back_right;
    private DcMotor linearSlideMotor_Right;
    private DcMotor linearSlideMotor_Left;
    private Servo intake;
    private Servo door;
    private Servo airplane;
    private Servo arm;



    private final int SLIDE_UP_POSITION = 1000;  // Adjust this value
    private final int SLIDE_DOWN_POSITION = 0;   // Adjust this value

    @Override
    public void runOpMode() {
        back_left = hardwareMap.dcMotor.get("back_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        front_left = hardwareMap.dcMotor.get("front_left");
        back_right = hardwareMap.dcMotor.get("back_right");
        linearSlideMotor_Right = hardwareMap.dcMotor.get("linearSlideMotor_right");
        linearSlideMotor_Left = hardwareMap.dcMotor.get("linearSlideMotor_left");
        intake = hardwareMap.servo.get("intake");
        door = hardwareMap.servo.get("door");
        airplane = hardwareMap.servo.get("airplane");
        arm = hardwareMap.servo.get("arm");
        //claw = hardwareMap.get(Servo.class, "claw");
        //door = hardwareMap.get(Servo.class, "door");

        linearSlideMotor_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideMotor_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideMotor_Right.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // Mecanum drive code
            double left_stick_x = -(gamepad1.left_stick_x);
            double left_stick_y = gamepad1.left_stick_y;
            double linear_motion = gamepad2.left_stick_y;
            double Theta = Math.atan2(left_stick_x, left_stick_y);
            double power = Math.hypot(left_stick_x, left_stick_y);
            double sin = Math.sin(Theta - Math.PI / 4);
            double cos = Math.cos(Theta - Math.PI / 4);
            double max = Math.max(Math.abs(cos), Math.abs(sin));
            double rx = gamepad1.right_stick_x / max;
            double leftfrontpower = cos * power + rx;
            double rightfrontpower = sin * power - rx;
            double leftrearpower = sin * power + rx;
            double rightrearpower = cos * power + rx;
            int maxExtention = 2200;


            if ((power + Math.abs(left_stick_x)) > 1) {
                leftfrontpower /= power + left_stick_x;
                rightfrontpower /= power + left_stick_x;
                leftrearpower /= power + left_stick_x;
                rightrearpower /= power + left_stick_x;
            }
            rx = gamepad1.right_stick_x / max;
            ///leftfrontpower = (rotX+rotY);
            ///rightfrontpower = (-rotX+rotY);
            ///leftrearpower = (-rotX+rotY);
            ///rightrearpower = (rotX+rotY);
            leftfrontpower = cos * power - rx;
            rightfrontpower = (sin * power - rx);
            leftrearpower = (sin * power + rx);
            rightrearpower = (cos * power + rx);
            front_left.setPower(Math.cbrt(leftfrontpower));
            back_left.setPower(Math.cbrt(-leftrearpower));
            front_right.setPower(Math.cbrt(-rightfrontpower));
            back_right.setPower(Math.cbrt(-rightrearpower));


            //new linnear slide code
            if (linearSlideMotor_Right.getCurrentPosition()<maxExtention && linearSlideMotor_Left.getCurrentPosition()<maxExtention){
                if (gamepad1.y){
                    telemetry.update();
                    linearSlideMotor_Right.setPower(1);
                    linearSlideMotor_Left.setPower(1);
                }
                else {
                    linearSlideMotor_Right.setPower(0.06);
                    linearSlideMotor_Left.setPower(0.06);
                }
            }
            else {
                linearSlideMotor_Right.setPower(0);
                linearSlideMotor_Left.setPower(0);
            }

            if (linearSlideMotor_Right.getCurrentPosition()>0 && linearSlideMotor_Left.getCurrentPosition()>0){
                if (gamepad1.a){
                    telemetry.update();
                    linearSlideMotor_Right.setPower(-0.5);
                    linearSlideMotor_Left.setPower(-0.5);
                }
                else {
                    linearSlideMotor_Right.setPower(0);
                    linearSlideMotor_Left.setPower(0);
                }
            }
            else {
                linearSlideMotor_Right.setPower(0);
                linearSlideMotor_Left.setPower(0);
            }
            if (gamepad2.dpad_right){
                door.setPosition(1);
            }
            if (gamepad2.dpad_left){
                door.setPosition(-1);
            }
            if (gamepad1.left_bumper){
                airplane.setPosition(-1);
            }
            if (gamepad1.right_bumper){
                airplane.setPosition(1);
            }

            if (gamepad2.dpad_up){
                arm.setPosition(-1);
            }

            if (gamepad2.dpad_down){
                arm.setPosition(0.5);
            }

            if (gamepad2.x){
                intake.setPosition(1);
            }

            if (gamepad2.b){
                intake.setPosition(.5);
            }


//            if (gamepad2.x) {
//                intake_wheel.setPower(1);
//            }
            // Telemetry for debugging
            telemetry.addData("Front Left Pow", front_left.getPower());
            telemetry.addData("Back Left Pow", back_left.getPower());
            telemetry.addData("Front Right Pow", front_right.getPower());
            telemetry.addData("Back Right Pow", back_right.getPower());
            telemetry.addData("Linear Slide Right Position", linearSlideMotor_Right.getCurrentPosition());
            telemetry.addData("Linear Slide Left Position", linearSlideMotor_Left.getCurrentPosition());
            telemetry.update();

        }}
}








