package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Webcam;

@Autonomous(group = "test")
public class CameraTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Webcam webcam = new Webcam(this);
        webcam.init(hardwareMap);

        waitForStart();
        switch (webcam.location()) {
            case LEFT:
                // ...
                break;
            case MIDDLE:
                // ...
                break;
            case RIGHT:
                // ...
                break;
            case NOT_FOUND:
                // ...
                break;
        }
        webcam.stopStreaming();
    }
}
