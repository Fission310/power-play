package org.firstinspires.ftc.teamcode.opmode.dev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Clamp;

@TeleOp (name = "Dev Arm & Clamp", group = "dev")
public class DevArmClamp extends LinearOpMode {

    private Arm arm = new Arm(this);
    private Clamp clamp = new Clamp(this);

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);
        clamp.init(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            arm.loop(gamepad1);
            clamp.loop(gamepad1);
        }
    }
}
