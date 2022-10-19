package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.UASensor;

@TeleOp (name = "Dev MXSensor", group = "dev")
public class DevUASensor extends LinearOpMode {

    private UASensor uaSensor = new UASensor(this, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        uaSensor.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            uaSensor.telemetry(telemetry);

            telemetry.update();
        }
    }
}
