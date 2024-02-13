package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HSV.LeftPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="Vision Test")
public class VisionTest extends OpMode {

    VisionPortal visionPortal;
    LeftPropProcessor leftPropProcessor;

    @Override
    public void init() {
        leftPropProcessor = new LeftPropProcessor();
        leftPropProcessor.setDetectionColor(true);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessors(leftPropProcessor)
                .build();
    }

    @Override
    public void loop() {
        telemetry.addData("Prop Location", leftPropProcessor.getPropLocation());
    }
}
