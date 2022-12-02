package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp (name = "TestMotorDirection", group = "dev")
public class TestMotorDirection extends LinearOpMode {

//    DcMotorEx leftSlideMotor;
    DcMotorEx rightSlideMotor;

    public static double POWER = 0.4;

    public void rotate() {
//        leftSlideMotor.setPower(POWER);
        rightSlideMotor.setPower(POWER);
    }

    public void reverse() {
        rightSlideMotor.setPower(-POWER);
    }

    public void stopRotation() {
//        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
//        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
//        leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.right_trigger > 0) {
                rotate();
            } else if (gamepad1.left_trigger > 0) {
                reverse();
            } else {
                stopRotation();
            }
        }
    }
}
