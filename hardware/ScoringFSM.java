package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SlidesFSM extends Mechanism {

    public static double DELAY_PREPARING = 0.2;
    public static double DELAY_LOW = 0.2;
    public static double DELAY_MEDIUM = 0.3;
    public static double DELAY_HIGH = 0.5;
    public static double DELAY_RETRACTING = 0.3;

    private SlidesMotors slidesMotors = new SlidesMotors(opMode);

    ElapsedTime time;

    public SlidesFSM(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        slidesMotors.init(hwMap);
        time = new ElapsedTime();
    }

    public enum SlidesState {
        REST,
        PREPARING,
        WAIT_TO_EXTEND,
        EXTENDING,
        WAIT_TO_RETRACT,
        RETRACTING
    }
    public SlidesState slidesState = SlidesState.REST;

    public enum SlidesLevel {
        LOW,
        MEDIUM,
        HIGH
    }
    public SlidesLevel slidesLevel = SlidesLevel.LOW;

    @Override
    public void loop(Gamepad gamepad) {

        if (gamepad.left_stick_button) {
            slidesMotors.rest();
            slidesState = SlidesState.REST;
        }

        slidesMotors.update();
        switch (slidesState) {
            case REST:
                slidesMotors.rest();
                slidesState = SlidesState.PREPARING;
                break;
            case PREPARING:
                if (gamepad.right_bumper) {
                    slidesMotors.extendPrepArm();
                    slidesState = SlidesState.WAIT_TO_EXTEND;
                    time.reset();
                }
                break;
            case WAIT_TO_EXTEND:
                if (time.seconds() > DELAY_PREPARING) {
                    if (gamepad.a) {
                        slidesMotors.extendLow();
                        slidesLevel = SlidesLevel.LOW;
                        slidesState = SlidesState.EXTENDING;
                        time.reset();
                    } else if (gamepad.b) {
                        slidesMotors.extendMedium();
                        slidesLevel = SlidesLevel.MEDIUM;
                        slidesState = SlidesState.EXTENDING;
                        time.reset();
                    } else if (gamepad.y) {
                        slidesMotors.extendHigh();
                        slidesLevel = SlidesLevel.HIGH;
                        slidesState = SlidesState.EXTENDING;
                        time.reset();
                    }
                }
                break;
            case EXTENDING:
                switch (slidesLevel) {
                    case LOW:
                        if (time.seconds() > DELAY_LOW) {
                            slidesState = SlidesState.WAIT_TO_RETRACT;
                        }
                        break;
                    case MEDIUM:
                        if (time.seconds() > DELAY_MEDIUM) {
                            slidesState = SlidesState.WAIT_TO_RETRACT;
                        }
                        break;
                    case HIGH:
                        if (time.seconds() > DELAY_HIGH) {
                            slidesState = SlidesState.WAIT_TO_RETRACT;
                        }
                        break;
                }
                break;
            case WAIT_TO_RETRACT:
                if (gamepad.dpad_down) {
                    slidesMotors.descendABit();
                }
                else if (gamepad.dpad_up) {
                    slidesMotors.ascendABit();
                }
                else if (gamepad.x || gamepad.left_bumper) {
                    slidesState = SlidesState.RETRACTING;
                    time.reset();
                }
                break;
            case RETRACTING:
                if (time.seconds() > DELAY_RETRACTING) {
                    slidesMotors.rest();
                    slidesState = SlidesState.REST;
                }
                break;
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Current state", slidesState);
        slidesMotors.telemetry(telemetry);
    }

}
