package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.ConeSensor;

@TeleOp (name = "Dev Cone Sensor", group = "dev")
public class DevConeSensor extends LinearOpMode {

    ConeSensor sensor = new ConeSensor(this);
    
    @Override
    public void runOpMode() throws InterruptedException {
        sensor.init(hardwareMap);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            sensor.telemetry(telemetry);
        }
    }
}
