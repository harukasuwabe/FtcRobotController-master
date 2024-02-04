package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.Servo;

public class Movement {

    // todo: write your code here
    // private static DcMotor motorFrontRight;
    // private static DcMotor motorFrontLeft;
    // private static DcMotor motorBackLeft;
    // private static DcMotor motorBackRight;
    // private static DcMotor ArmMotor;
    // private static Servo claw;

    static double rotations;
    static double ticks;


    public static void left(double distance, Telemetry telemetry, DcMotor motorBackLeft, DcMotor motorBackRight, DcMotor motorFrontLeft, DcMotor motorFrontRight) {
        reset_encoders(motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight);
        rotations = distance * 21.74;
        // SET TARGET POSITION
        motorFrontRight.setTargetPosition((int) rotations);
        motorFrontLeft.setTargetPosition((int) -rotations);
        motorBackLeft.setTargetPosition((int) rotations);
        motorBackRight.setTargetPosition((int) -rotations);
        // RUN TO POSITION
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // POWER (left slide)
        ((DcMotorEx) motorFrontRight).setVelocity(1000);
        ((DcMotorEx) motorFrontLeft).setVelocity(-1000);
        ((DcMotorEx) motorBackLeft).setVelocity(1000);
        ((DcMotorEx) motorBackRight).setVelocity(-1000);
        while (motorBackLeft.isBusy() && motorBackRight.isBusy() && motorFrontLeft.isBusy() && motorFrontRight.isBusy()) {
            telemetry.addData("function", "left");
            motor_telemetry(telemetry, motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight);
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    public static void forward(int distance, Telemetry telemetry, DcMotor motorBackLeft, DcMotor motorBackRight, DcMotor motorFrontLeft, DcMotor motorFrontRight) {
        reset_encoders(motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight);
        rotations = distance * 21.74;
        // SET TARGET POSITION
        motorFrontRight.setTargetPosition((int) rotations);
        motorFrontLeft.setTargetPosition((int) rotations);
        motorBackLeft.setTargetPosition((int) rotations);
        motorBackRight.setTargetPosition((int) rotations);
        // RUN TO POSITION
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // POWER (left slide)
        ((DcMotorEx) motorFrontRight).setVelocity(1000);
        ((DcMotorEx) motorFrontLeft).setVelocity(1000);
        ((DcMotorEx) motorBackLeft).setVelocity(1000);
        ((DcMotorEx) motorBackRight).setVelocity(1000);
        while (motorBackLeft.isBusy() && motorBackRight.isBusy() && motorFrontLeft.isBusy() && motorFrontRight.isBusy()) {
            telemetry.addData("function", "forward...");
            motor_telemetry(telemetry, motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight);
            telemetry.update();
        }
    }


    /**
     * Describe this function...
     */
    public static void backward(double distance, Telemetry telemetry, DcMotor motorBackLeft, DcMotor motorBackRight, DcMotor motorFrontLeft, DcMotor motorFrontRight) {
        reset_encoders(motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight);
        rotations = distance * 21.74;
        // SET TARGET POSITION
        motorFrontRight.setTargetPosition((int) -rotations);
        motorFrontLeft.setTargetPosition((int) -rotations);
        motorBackLeft.setTargetPosition((int) -rotations);
        motorBackRight.setTargetPosition((int) -rotations);
        // RUN TO POSITION
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // POWER (left slide)
        ((DcMotorEx) motorFrontRight).setVelocity(-1000);
        ((DcMotorEx) motorFrontLeft).setVelocity(-1000);
        ((DcMotorEx) motorBackLeft).setVelocity(-1000);
        ((DcMotorEx) motorBackRight).setVelocity(-1000);
        while (motorBackLeft.isBusy() && motorBackRight.isBusy() && motorFrontLeft.isBusy() && motorFrontRight.isBusy()) {
            telemetry.addData("function", "backward");
            motor_telemetry(telemetry, motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight);
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    public static void right(double distance, Telemetry telemetry, DcMotor motorBackLeft, DcMotor motorBackRight, DcMotor motorFrontLeft, DcMotor motorFrontRight) {
        reset_encoders(motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight);
        rotations = distance * 21.74;
        // SET TARGET POSITION
        motorFrontRight.setTargetPosition((int) -rotations);
        motorFrontLeft.setTargetPosition((int) rotations);
        motorBackLeft.setTargetPosition((int) -rotations);
        motorBackRight.setTargetPosition((int) rotations);
        // RUN TO POSITION
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // POWER (left slide)
        ((DcMotorEx) motorFrontRight).setVelocity(-1000);
        ((DcMotorEx) motorFrontLeft).setVelocity(1000);
        ((DcMotorEx) motorBackLeft).setVelocity(-1000);
        ((DcMotorEx) motorBackRight).setVelocity(1000);
        while (motorBackLeft.isBusy() && motorBackRight.isBusy() && motorFrontLeft.isBusy() && motorFrontRight.isBusy()) {
            telemetry.addData("function", "right");
            motor_telemetry(telemetry, motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight);
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    public static void reset_encoders(DcMotor motorBackLeft, DcMotor motorBackRight, DcMotor motorFrontLeft, DcMotor motorFrontRight) {
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public static void reset_linear_encoders(DcMotor motorLinearSlideLeft, DcMotor motorLinearSlideRight) {
        motorLinearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLinearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    /**
     * Describe this function...
     */
    public static void motor_telemetry(Telemetry telemetry, DcMotor motorBackLeft, DcMotor motorBackRight, DcMotor motorFrontLeft, DcMotor motorFrontRight) {
        telemetry.addData("back left pos", motorBackLeft.getCurrentPosition());
        telemetry.addData("back right pos", motorBackRight.getCurrentPosition());
        telemetry.addData("front left pos", motorFrontLeft.getCurrentPosition());
        telemetry.addData("front right pos", motorFrontRight.getCurrentPosition());
        telemetry.update();


    }

    //rotation right
    public static void rotationRight(double degrees, Telemetry telemetry, DcMotor motorBackLeft, DcMotor motorBackRight, DcMotor motorFrontLeft, DcMotor motorFrontRight) {
        reset_encoders(motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight);
        rotations = degrees * 13;
        // SET TARGET POSITION
        motorFrontRight.setTargetPosition((int) -rotations);
        motorFrontLeft.setTargetPosition((int) rotations);
        motorBackLeft.setTargetPosition((int) rotations);
        motorBackRight.setTargetPosition((int) -rotations);
        // RUN TO POSITION
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // POWER (left slide)
        ((DcMotorEx) motorFrontRight).setVelocity(-1000);
        ((DcMotorEx) motorFrontLeft).setVelocity(1000);
        ((DcMotorEx) motorBackLeft).setVelocity(1000);
        ((DcMotorEx) motorBackRight).setVelocity(-1000);
        while (motorBackLeft.isBusy() && motorBackRight.isBusy() && motorFrontLeft.isBusy() && motorFrontRight.isBusy()) {
            telemetry.addData("function", "rotation right");
            motor_telemetry(telemetry, motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight);
            telemetry.update();
        }
    }

    public static void rotationLeft(double degrees, Telemetry telemetry, DcMotor motorBackLeft, DcMotor motorBackRight, DcMotor motorFrontLeft, DcMotor motorFrontRight) {
        reset_encoders(motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight);
        rotations = degrees * 13;
        // SET TARGET POSITION
        motorFrontRight.setTargetPosition((int) rotations);
        motorFrontLeft.setTargetPosition((int) -rotations);
        motorBackLeft.setTargetPosition((int) -rotations);
        motorBackRight.setTargetPosition((int) rotations);
        // RUN TO POSITION
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // POWER (left slide)
        ((DcMotorEx) motorFrontRight).setVelocity(1000);
        ((DcMotorEx) motorFrontLeft).setVelocity(-1000);
        ((DcMotorEx) motorBackLeft).setVelocity(-1000);
        ((DcMotorEx) motorBackRight).setVelocity(1000);
        while (motorBackLeft.isBusy() && motorBackRight.isBusy() && motorFrontLeft.isBusy() && motorFrontRight.isBusy()) {
            telemetry.addData("function", "rotation right");
            motor_telemetry(telemetry, motorBackLeft, motorBackRight, motorFrontLeft, motorFrontRight);
            telemetry.update();
        }
    }
    public static void linearSlides(int ticks, Telemetry telemetry, DcMotor motorLinearSlideLeft, DcMotor motorLinearSlideRight){
        reset_linear_encoders(motorLinearSlideLeft, motorLinearSlideRight);
        //SET TARGET POSITION
        motorLinearSlideLeft.setTargetPosition((int)ticks);
        motorLinearSlideRight.setTargetPosition((int)ticks);
        //RUN TO POSITION
        motorLinearSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLinearSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //POWER
        ((DcMotorEx) motorLinearSlideLeft).setVelocity(200);
        ((DcMotorEx) motorLinearSlideRight).setVelocity(-200);
    }
}






