package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.hardware.drivebase.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.SlidesMotors;

@TeleOp (name = "FIX Slides", group = "_zmain")
@Config
public class FixSlides extends LinearOpMode {

    private Drivetrain drivetrain = new Drivetrain(this);
    private SlidesMotors slidesMotors = new SlidesMotors(this);
    private Arm arm = new Arm(this);

    public static double UP_POWER = 0.5;
    public static double DOWN_POWER = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        slidesMotors.init(hardwareMap);
        arm.init(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drivetrain.loop(gamepad1);

            if (gamepad1.right_trigger > 0) {
                slidesMotors.setPower(UP_POWER);
            } else if (gamepad1.left_trigger > 0) {
                slidesMotors.setPower(-DOWN_POWER);
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
