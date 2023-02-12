package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.auton.AutoConstants;

@Config
public class ScoringFSM extends Mechanism {

    // TODO: try decreasing the delay :100:
    public static double DELAY_PREPARING = 0.2;

    public static double DELAY_GROUND = 0.2;
    public static double DELAY_LOW = 0.2;
    public static double DELAY_MEDIUM = 0.3;
    public static double DELAY_HIGH = 0.5;
    public static double DELAY_RETRACTING = 0.3;
    public static double DELAY_SCORING = 0.3;

    public static double DELAY_CLAMP_INTAKE = 1;

    private SlidesMotors slidesMotors = new SlidesMotors(opMode);
    private Arm arm = new Arm(opMode);
    private Clamp clamp = new Clamp(opMode);
    private ConeSensor coneSensor = new ConeSensor(opMode);

    private ElapsedTime time;
    private boolean clampOverride;

    private int cycleConeStack = 1;

    public ScoringFSM(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        slidesMotors.init(hwMap);
        arm.init(hwMap);
        clamp.init(hwMap);
        coneSensor.init(hwMap);

        time = new ElapsedTime();
        clampOverride = false;
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

    // AUTOMATIC scoring mode will raise slides to high position without confirmation
    public enum ScoringMode {
        MANUAL,
        AUTOMATIC
    }
    public ScoringMode scoringMode = ScoringMode.MANUAL;

    boolean isPressedLeftTrigger = false;
    boolean isPressedRightStickButton = false;

    @Override
    public void loop(Gamepad gamepad) {

        if (gamepad.dpad_left) {
            clampOverride = false;
            clamp.intakePos();
            slidesState = SlidesState.PREPARING;
        } else if (gamepad.dpad_right) {
            clampOverride = true;
            clamp.close();
            time.reset();
            slidesState = SlidesState.PREPARING;
        }

        else if (gamepad.left_stick_button) {
            time.reset();
            slidesMotors.teleRest();
            slidesState = SlidesState.REST;
        }
        // toggles between scoring modes
        else if (!isPressedRightStickButton && gamepad.right_stick_button) {
            switch (scoringMode) {
                case MANUAL:
                    scoringMode = ScoringMode.AUTOMATIC;
                    break;
                case AUTOMATIC:
                    scoringMode = ScoringMode.MANUAL;
            }
        }

        else if (!isPressedLeftTrigger && gamepad.left_trigger > 0) {
            slidesMotors.setTeleRestPos(AutoConstants.SLIDE_EXTEND_POSITIONS[cycleConeStack]);
            time.reset();
            slidesMotors.teleRest();
            slidesState = SlidesState.REST;
            cycleConeStack += 1;
        } else if (gamepad.right_trigger > 0) {
            slidesMotors.setTeleRestPos(0);
            time.reset();
            slidesMotors.teleRest();
            slidesState = SlidesState.REST;
            cycleConeStack = 1;
        }

        isPressedLeftTrigger = gamepad.left_trigger > 0;
        isPressedRightStickButton = gamepad.right_stick_button;

        slidesMotors.update();

        switch (slidesState) {
            case REST:
                slidesMotors.teleRest();
                arm.intakePos();
                clamp.open();

                slidesState = SlidesState.PREPARING;
                time.reset();
                break;
            case PREPARING:
                if ((time.seconds() > DELAY_CLAMP_INTAKE) && !clampOverride) {
                    clamp.intakePos();
                }
                if (clampOverride) {
                    if (time.seconds() > DELAY_GROUND) {
                        arm.groundScorePos();
                    }
                }
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

                    switch (scoringMode) {
                        case MANUAL:
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
                            break;
                        case AUTOMATIC:
                            slidesMotors.extendHigh();
                            slidesLevel = SlidesLevel.HIGH;
                            slidesState = SlidesState.EXTENDING;
                            time.reset();
                            break;
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
                            slidesMotors.extendToPosition(slidesMotors.getPosition() - SlidesMotors.TELE_DROP_AMT);
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
                    clamp.open();
                    arm.intakePos();
                    slidesMotors.teleRest();
                    slidesState = SlidesState.REST;
                    time.reset();
                }
                break;
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("restPos", slidesMotors.getTeleRestPos());
    }

}
