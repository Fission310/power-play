package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

@Config
public class Intake extends Mechanism {

    CRServo leftIntake;
    CRServo rightIntake;

    public static double LEFT_POWER = -1;
    public static double RIGHT_POWER = 1;

    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        leftIntake = hwMap.get(CRServo.class, "leftIntake");
        rightIntake = hwMap.get(CRServo.class, "rightIntake");

        // leftIntake spins counterclockwise
        // rightIntake spins clockwise
    }

    public void intake() {
        leftIntake.setPower(LEFT_POWER);
        rightIntake.setPower(RIGHT_POWER);
    }

    public void stop() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    public void outtake() {
        leftIntake.setPower(-LEFT_POWER);
        rightIntake.setPower(-RIGHT_POWER);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.right_trigger > 0) {
            intake();
        } else if (gamepad.left_trigger > 0) {
            outtake();
        } else {
            stop();
        }
    }

}
