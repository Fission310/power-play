package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SlidesFSM extends Mechanism {

    public static long RETRACT_STATE_DELAY = 100;

    private SlidesMotors slidesMotors = new SlidesMotors(opMode);

    private Thread lowThread;
    private Thread mediumThread;
    private Thread highThread;

    public SlidesFSM(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        slidesMotors.init(hwMap);

        lowThread = new Thread(low);
        mediumThread = new Thread(medium);
        highThread = new Thread(high);
    }

    public enum SlidesState {
        REST,
        WAIT_EXTEND,
        WAIT_RETRACT
    }
    public SlidesState slidesState = SlidesState.REST;

    public Runnable low = () -> {
        try {
            slidesMotors.extendLow();
            Thread.sleep(RETRACT_STATE_DELAY);
            slidesState = SlidesState.WAIT_RETRACT;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable medium = () -> {
      try {
          slidesMotors.extendMedium();
          Thread.sleep(RETRACT_STATE_DELAY);
          slidesState = SlidesState.WAIT_RETRACT;
      } catch (InterruptedException e) {
          e.printStackTrace();
      }
    };

    public Runnable high = () -> {
      try {
          slidesMotors.extendHigh();
          Thread.sleep(RETRACT_STATE_DELAY);
          slidesState = SlidesState.WAIT_RETRACT;
      } catch (InterruptedException e) {
          e.printStackTrace();
      }
    };

    @Override
    public void loop(Gamepad gamepad) {
        slidesMotors.update();
        switch (slidesState) {
            case REST:
                slidesMotors.rest();
                slidesState = SlidesState.WAIT_EXTEND;
                break;
            case WAIT_EXTEND:
                if (gamepad.a) {
                    try {
                        lowThread.start();
                    } catch (IllegalThreadStateException ignored) {}
                }
                if (gamepad.b) {
                    try {
                        mediumThread.start();
                    } catch (IllegalThreadStateException ignored) {}
                }
                if (gamepad.y) {
                    try {
                        highThread.start();
                    } catch (IllegalThreadStateException ignored) {}
                }
                break;
            case WAIT_RETRACT:
                if (gamepad.x) {
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
