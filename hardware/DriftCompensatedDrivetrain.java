package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class DriftCompensatedDrivetrain extends Mechanism {

    SampleMecanumDrive drive;

    public static double FRONT_LEFT_MULTIPLIER = 1;
    public static double BACK_LEFT_MULTIPLIER = 1;
    public static double BACK_RIGHT_MULTIPLIER = 0.99;
    public static double FRONT_RIGHT_MULTIPLIER = 0.98;

    public DriftCompensatedDrivetrain(LinearOpMode opMode) { this.opMode = opMode; }

    public void init(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop(Gamepad gamepad) {
        double y = -gamepad.left_stick_y; // Remember, this is reversed!
        double x = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        drive.setMotorPowers(frontLeftPower * FRONT_LEFT_MULTIPLIER, backLeftPower * BACK_LEFT_MULTIPLIER, backRightPower * BACK_RIGHT_MULTIPLIER, frontRightPower * FRONT_RIGHT_MULTIPLIER);
    }

}
