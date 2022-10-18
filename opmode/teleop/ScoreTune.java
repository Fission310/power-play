package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.MeccRobot;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.slides.Carriage;

@TeleOp (name = "ScoreTune", group = "test")
public class ScoreTune extends LinearOpMode {

    private Carriage carriage = new Carriage(this);

    @Override
    public void runOpMode() throws InterruptedException {
        carriage.init(hardwareMap);

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            carriage.loop(gamepad1);
//            carriage.telemetry(telemetry);

//            telemetry.update();
        }
    }

}
