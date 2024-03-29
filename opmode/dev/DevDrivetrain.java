package org.firstinspires.ftc.teamcode.opmode.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.drivebase.Drivetrain;

@TeleOp (name = "Dev Drivetrain", group = "dev")
public class DevDrivetrain extends LinearOpMode {

    private Drivetrain dt = new Drivetrain(this);

    @Override
    public void runOpMode() throws InterruptedException {
        dt.init(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            dt.loop(gamepad1);
        }
    }
}
