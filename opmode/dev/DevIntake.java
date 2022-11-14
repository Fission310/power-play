package org.firstinspires.ftc.teamcode.opmode.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Intake;

@TeleOp (name = "Dev Intake", group = "dev")
public class DevIntake extends LinearOpMode {

    private Intake intake = new Intake(this);

    @Override
    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            intake.loop(gamepad1);
        }
    }
}
