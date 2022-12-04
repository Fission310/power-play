package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.stuyfission.fissionlib.util.Mechanism;

@Config
public class Arm extends Mechanism {

    private Servo leftArm;
    private Servo rightArm;

    public static double INTAKE_POS = 0.04;
    public static double SCORE_POS = 0.72;

    public Arm(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        leftArm = hwMap.get(Servo.class, "leftArm");
        rightArm = hwMap.get(Servo.class, "rightArm");

        rightArm.setDirection(Servo.Direction.REVERSE);
    }

    public void intakePos() {
        leftArm.setPosition(INTAKE_POS);
        rightArm.setPosition(INTAKE_POS);
    }

    public void scorePos() {
        leftArm.setPosition(SCORE_POS);
        rightArm.setPosition(SCORE_POS);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            scorePos();
        } else if (gamepad.dpad_down) {
            intakePos();
        }
    }

}
