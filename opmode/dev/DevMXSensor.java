package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.MXSensor;

@TeleOp (name = "Dev MXSensor", group = "dev")
public class DevMXSensor extends LinearOpMode {

    private MXSensor mxSensor = new MXSensor(this, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        mxSensor.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            mxSensor.telemetry(telemetry);

            telemetry.update();
        }
    }
}
