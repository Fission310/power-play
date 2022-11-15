package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.SlidesFSM;

@TeleOp (name = "Dev Slides", group = "dev")
public class DevSlides extends LinearOpMode {

    private SlidesFSM slidesFSM = new SlidesFSM(this);

    @Override
    public void runOpMode() throws InterruptedException {
        slidesFSM.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            slidesFSM.loop(gamepad1);

            slidesFSM.telemetry(telemetry);
            telemetry.update();
        }
    }
}
