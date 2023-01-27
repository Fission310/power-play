package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ScoringFSM extends Mechanism {

    public static double DELAY_PREPARING = 0.2;
    public static double DELAY_GROUND = 0.2;
    public static double DELAY_LOW = 0.2;
    public static double DELAY_MEDIUM = 0.3;
    public static double DELAY_HIGH = 0.5;
    public static double DELAY_RETRACTING = 0.3;
    public static double DELAY_SCORING = 0.3;

    private SlidesMotors slidesMotors = new SlidesMotors(opMode);
    private Arm arm = new Arm(opMode);
    private Clamp clamp = new Clamp(opMode);
//    private ConeSensor coneSensor = new ConeSensor(opMode);

    ElapsedTime time;

    public ScoringFSM(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        slidesMotors.init(hwMap);
        arm.init(hwMap);
        clamp.init(hwMap);
//        coneSensor.init(hwMap);

        time = new ElapsedTime();
    }

    public enum SlidesState {
        REST,
        PREPARING,
        WAIT_TO_EXTEND,
        EXTENDING,
        WAIT_TO_RETRACT,
        SCORING,
        GROUND_JUNCTION,
        RETRACTING
    }
    public SlidesState slidesState = SlidesState.REST;

    public enum SlidesLevel {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }
    public SlidesLevel slidesLevel = SlidesLevel.LOW;

    @Override
    public void loop(Gamepad gamepad) {

        if (gamepad.dpad_left) {
            clamp.open();
        } else if (gamepad.dpad_right) {
            clamp.close();
        }

        else if (gamepad.left_stick_button) {
            slidesMotors.rest();
            slidesState = SlidesState.REST;
        }

        slidesMotors.update();
        switch (slidesState) {
            case REST:
                slidesMotors.rest();
                arm.intakePos();
                clamp.open();

                slidesState = SlidesState.PREPARING;
                break;
            case PREPARING:
                if (gamepad.right_bumper) {
                    clamp.close();
                    slidesMotors.extendPrepArm();
                    slidesState = SlidesState.WAIT_TO_EXTEND;
                    time.reset();
                }
                break;
            case WAIT_TO_EXTEND:
                if (time.seconds() > DELAY_PREPARING) {

                    arm.scorePos();

                    if (gamepad.x) {
                        slidesMotors.extendGround();
                        arm.groundScorePos();
                        slidesLevel = SlidesLevel.GROUND;
                        slidesState = SlidesState.EXTENDING;
                        time.reset();
                    } else if (gamepad.a) {
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
                    case GROUND:
                        if (time.seconds() > DELAY_GROUND) {
                            slidesState = SlidesState.WAIT_TO_RETRACT;
                        }
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
                switch (slidesLevel) {
                    case GROUND:
                        if (gamepad.x || gamepad.left_bumper) {
                            clamp.open();
                            slidesState = SlidesState.GROUND_JUNCTION;
                            time.reset();
                        }
                        break;
                    case LOW:
                    case MEDIUM:
                    case HIGH:
                        if (gamepad.dpad_down) {
                            slidesMotors.descendABit();
                        } else if (gamepad.dpad_up) {
                            slidesMotors.ascendABit();
                        } else if (gamepad.x || gamepad.left_bumper) {
                            // TESTS //
                            slidesMotors.extendToPosition(slidesMotors.getPosition() - 9);
                            slidesState = SlidesState.SCORING;
                            time.reset();
                            // END TESTS //

                            // HOW IT WAS BEFORE //
//                            clamp.open();
//                            slidesState = SlidesState.RETRACTING;
//                            time.reset();
                            // END HOW IT WAS BEFORE //
                        }
                        break;
                }
                break;
            case SCORING:
                if (time.seconds() > DELAY_SCORING) {
                    clamp.open();
                    slidesState = SlidesState.RETRACTING;
                    time.reset();
                }
                break;
            case GROUND_JUNCTION:
                if (time.seconds() > DELAY_RETRACTING) {
                    slidesMotors.extendPrepArm();
                    time.reset();
                    slidesState = SlidesState.RETRACTING;
                }
                break;
            case RETRACTING:
                if (time.seconds() > DELAY_RETRACTING) {
                    arm.intakePos();
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
