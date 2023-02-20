package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ScoringFSM;

@TeleOp (name = "Dev Slides", group = "dev")
public class DevSlides extends LinearOpMode {

    private ScoringFSM scoringFSM = new ScoringFSM(this);

    @Override
    public void runOpMode() throws InterruptedException {
        scoringFSM.init(hardwareMap);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            scoringFSM.loop(gamepad1);

            scoringFSM.telemetry(telemetry);
            telemetry.update();
        }
    }
}
