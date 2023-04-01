package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.ConeStackWebcam;

@TeleOp (name = "Dev Cone Stack Webcam", group = "dev")
public class DevConeStackWebcam extends LinearOpMode {
    private ConeStackWebcam coneStackWebcam = new ConeStackWebcam(this);
    
    @Override
    public void runOpMode() throws InterruptedException {
        coneStackWebcam.init(hardwareMap);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        coneStackWebcam.stopStreaming();
    }
}
