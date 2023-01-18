package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.SlidesMotors;

@TeleOp (name = "FIX Slides", group = "_zmain")
public class FixSlides extends LinearOpMode {

    private Drivetrain drivetrain = new Drivetrain(this);
    private SlidesMotors slidesMotors = new SlidesMotors(this);
    private Arm arm = new Arm(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        slidesMotors.init(hardwareMap);
        arm.init(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drivetrain.loop(gamepad1);

            if (gamepad1.right_trigger > 0) {
                slidesMotors.setPower(0.5);
            } else if (gamepad1.left_trigger > 0) {
                slidesMotors.setPower(-0.5);
            } else if (gamepad1.dpad_up) {
                arm.scorePos();
            } else if (gamepad1.dpad_down) {
                arm.intakePos();
            } else {
                slidesMotors.setPower(0);
            }
        }
    }

}
