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

    public static double INTAKE_POS = 0.84; // 0.18
    public static double AUTO_INTAKE_POS = 0.18;
    public static double AUTO_SCORE_POS = 0.28;
    public static double AUTO_CONE_STACK_POS = 0.57;
    public static double SCORE_POS = 0.3; //0.76
    public static double GROUND_SCORE_POS = 0.79;

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

    public void autoIntakePos() {
        leftArm.setPosition(AUTO_INTAKE_POS);
        rightArm.setPosition(AUTO_INTAKE_POS);
    }

    public void autoScorePos() {
        leftArm.setPosition(AUTO_SCORE_POS);
        rightArm.setPosition(AUTO_SCORE_POS);
    }

    public void autoConeStackPos() {
        leftArm.setPosition(AUTO_CONE_STACK_POS);
        rightArm.setPosition(AUTO_CONE_STACK_POS);
    }

    public void scorePos() {
        leftArm.setPosition(SCORE_POS);
        rightArm.setPosition(SCORE_POS);
    }

    public void groundScorePos() {
        leftArm.setPosition(GROUND_SCORE_POS);
        rightArm.setPosition(GROUND_SCORE_POS);
    }

    public void moveToPos(double pos) {
        leftArm.setPosition(pos);
        rightArm.setPosition(pos);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            scorePos();
        } else if (gamepad.dpad_down) {
            intakePos();
        } else if (gamepad.dpad_right) {
            autoScorePos();
        } else if (gamepad.dpad_left) {
            autoConeStackPos();
        }
    }

}
