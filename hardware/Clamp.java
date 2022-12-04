package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.stuyfission.fissionlib.util.Mechanism;

@Config
public class Clamp extends Mechanism {

    private Servo clamp;
//    private CRServo crServo;

    public static double OPEN_POS = 0;
    public static double CLOSE_POS = 1;

    public Clamp(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        clamp = hwMap.get(Servo.class, "clamp");
        clamp.setDirection(Servo.Direction.REVERSE);

    }

    public void open() {
        clamp.setPosition(OPEN_POS);
    }

    public void close() {
        clamp.setPosition(CLOSE_POS);
//        crServo.setPower(0);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            close();
        } else if (gamepad.left_bumper) {
            open();
        }
    }

}
