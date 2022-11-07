package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Webcam;

@TeleOp (name = "Dev Webcam", group = "dev")
public class DevWebcam extends LinearOpMode {

    private Webcam webcam = new Webcam(this);

    @Override
    public void runOpMode() throws InterruptedException {
        webcam.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        switch (webcam.side()) {
            case ONE:
                break;
            case TWO:
                break;
            case THREE:
                break;
            case NOT_FOUND:
                break;
        }
        webcam.stopStreaming();

//        while (opModeIsActive() && !isStopRequested()) {
//            webcam.telemetry(telemetry);
//
//            telemetry.update();
//        }
    }
}
