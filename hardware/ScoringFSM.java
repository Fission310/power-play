package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ScoringFSM extends Mechanism {

    public static long ARM_TO_SCORE_DELAY = 250; // milliseconds
    public static long ARM_TO_RESET_DELAY = 200; // milliseconds

    private Arm arm = new Arm(opMode);
    private Clamp clamp = new Clamp(opMode);
    private ConeSensor coneSensor = new ConeSensor(opMode);

    private Thread scoreReadyThread;
    private Thread scoreResetThread;

    public ScoringFSM(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        arm.init(hwMap);
        clamp.init(hwMap);
        coneSensor.init(hwMap);

        scoreReadyThread = new Thread(scoreReady);
        scoreResetThread = new Thread(scoreReset);
    }

    public enum ScoreState {
        REST,
        WAIT_INTAKE,
        SCORE,
        SCORING
    }
    public static ScoreState scoreState = ScoreState.REST;

    public Runnable scoreReady = () -> {
      try {
          clamp.close();
          Thread.sleep(ARM_TO_SCORE_DELAY);
          scoreState = ScoreState.SCORE;
      } catch (InterruptedException e) {
          e.printStackTrace();
      }
    };

    public Runnable scoreReset = () -> {
        try {
            Thread.sleep(ARM_TO_RESET_DELAY);
            scoreState = ScoreState.REST;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public void runThread(Thread thread) {
        try {
            thread.start();
        } catch (IllegalThreadStateException ignored) {}
    }

    @Override
    public void loop(Gamepad gamepad) {
        switch (scoreState) {
            case REST:
                arm.intakePos();
                clamp.open();
                scoreState = ScoreState.WAIT_INTAKE;
                break;
            case WAIT_INTAKE:
                if (gamepad.left_bumper) {
                    clamp.open();
                } else if (gamepad.right_bumper) {
                    clamp.close();
                } else if (gamepad.dpad_up) {
                    runThread(scoreReadyThread);
                }
                else if (coneSensor.hasCone()) {
                    runThread(scoreReadyThread);
                }
                break;
            case SCORE:
                clamp.close();
                arm.scorePos();

                if (gamepad.x) {
                    clamp.open();
                    runThread(scoreResetThread);
                    scoreState = ScoreState.SCORING;
                }
                break;
            case SCORING:
                clamp.open();
                break;
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Current state", scoreState);
//        coneSensor.telemetry(telemetry);
    }

}
