package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.ConeSensor;

@TeleOp (name = "Dev Cone Sensor", group = "dev")
public class DevConeSensor extends LinearOpMode {

    ConeSensor sensor = new ConeSensor(this);

    @Override
    public void runOpMode() throws InterruptedException {
        sensor.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            sensor.telemetry(telemetry);
        }
    }
}
