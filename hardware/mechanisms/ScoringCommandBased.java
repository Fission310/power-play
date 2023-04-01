package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandMachine;
import com.stuyfission.fissionlib.command.CommandSequence;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

@SuppressWarnings("FieldMayBeFinal")
public class ScoringCommandBased extends Mechanism {

    private SlidesMotors slidesMotors = new SlidesMotors(opMode);
    private Arm arm = new Arm(opMode);
    private Clamp clamp = new Clamp(opMode);

    public ScoringCommandBased(LinearOpMode opMode) { this.opMode = opMode; }

    ////
    private Command clampCone = () -> {
        clamp.close();
        slidesMotors.extendPrepArm();
    };
    private Command prepareArm = () -> arm.scorePos();
    private CommandSequence clampSequence = new CommandSequence()
            .addCommand(clampCone)
            .addWaitCommand(ScoringFSM.DELAY_SCORING)
            .addCommand(prepareArm)
            .build();
    ////

    ////
    private Command dropSlides = () -> slidesMotors.extendToPosition(slidesMotors.getPosition() - SlidesMotors.TELE_DROP_AMT);
    private Command openClamp = () -> clamp.open();
    private Command retractSlides = () -> {
        arm.intakePos();
        slidesMotors.teleRest();
    };
    private Command intakePosClamp = () -> clamp.intakePos();
    private CommandSequence scoreSequence = new CommandSequence()
            .addCommand(dropSlides)
            .addWaitCommand(ScoringFSM.DELAY_SCORING)
            .addCommand(openClamp)
            .addWaitCommand(ScoringFSM.DELAY_RETRACTING)
            .addCommand(retractSlides)
            .addWaitCommand(0.35)
            .addCommand(intakePosClamp)
            .build();
    ////

    private CommandSequence resetSequence = new CommandSequence()
            .addCommand(openClamp)
            .addCommand(retractSlides)
            .addWaitCommand(0.35)
            .addCommand(intakePosClamp)
            .build();

    private CommandMachine commandMachine = new CommandMachine()
            .addCommandSequence(clampSequence, GamepadStatic.Input.RIGHT_BUMPER)
            .addCommandSequence(scoreSequence, GamepadStatic.Input.LEFT_BUMPER)
            .build();

    @Override
    public void init(HardwareMap hwMap) {
        slidesMotors.init(hwMap);
        slidesMotors.setMotionConstraints(200);
        arm.init(hwMap);
        clamp.init(hwMap);

        arm.intakePos();
        clamp.intakePos();
    }

    @Override
    public void loop(Gamepad gamepad) {
            slidesMotors.update();
            commandMachine.run(gamepad);

            if (gamepad.left_stick_button) {
                slidesMotors.teleRest();
                arm.intakePos();
                commandMachine.reset();
                resetSequence.trigger();
            }

            if (commandMachine.getCurrentCommandIndex() == 1) {
                if (gamepad.a) {
                    slidesMotors.extendLow();
                } else if (gamepad.b) {
                    slidesMotors.extendMedium();
                } else if (gamepad.y) {
                    slidesMotors.extendHigh();
                } else if (gamepad.x) {
                    arm.groundScorePos();
                    slidesMotors.extendGround();
                }
            }
    }

}
