package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.SignalSleeveWebcam;

@TeleOp (name = "Dev Signal Sleeve Webcam", group = "dev")
public class DevSignalSleeveWebcam extends LinearOpMode {

    private SignalSleeveWebcam signalSleeveWebcam = new SignalSleeveWebcam(this, "rightWebcam", SignalSleeveWebcam.ROBOT_SIDE.CONTROL_HUB);

    @Override
    public void runOpMode() throws InterruptedException {
        signalSleeveWebcam.init(hardwareMap);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        switch (signalSleeveWebcam.side()) {
            case ONE:
                break;
            case TWO:
                break;
            case THREE:
                break;
            case NOT_FOUND:
                break;
        }
        signalSleeveWebcam.stopStreaming();

//        while (opModeIsActive() && !isStopRequested()) {
//            webcam.telemetry(telemetry);
//
//            telemetry.update();
//        }
    }
}
