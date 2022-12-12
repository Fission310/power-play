package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SlidesFSM extends Mechanism {

    public static long EXTEND_ON_SENSOR_DELAY = 150;
    public static long RETRACT_STATE_DELAY = 100;
    public static long RETRACT_DELAY = 250;

    private SlidesMotors slidesMotors = new SlidesMotors(opMode);
    private ConeSensor coneSensor = new ConeSensor(opMode);

    private Thread prepArmThread;
    private Thread lowThread;
    private Thread mediumThread;
    private Thread highThread;
    private Thread retractThread;

    public SlidesFSM(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        slidesMotors.init(hwMap);
        coneSensor.init(hwMap);

        prepArmThread = new Thread(prepArm);
        lowThread = new Thread(low);
        mediumThread = new Thread(medium);
        highThread = new Thread(high);
        retractThread = new Thread(retract);
    }

    public enum SlidesState {
        REST,
        WAIT_EXTEND,
        WAIT_RETRACT
    }
    public SlidesState slidesState = SlidesState.REST;

    public Runnable prepArm = () -> {
        try {
            Thread.sleep(EXTEND_ON_SENSOR_DELAY);
            slidesMotors.extendPrepArm();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

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

    public Runnable retract = () -> {
        try {
            Thread.sleep(0);
            slidesMotors.rest();
            slidesState = SlidesState.REST;
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
        slidesMotors.update();
        switch (slidesState) {
            case REST:
                slidesMotors.rest();
                slidesState = SlidesState.WAIT_EXTEND;
                break;
            case WAIT_EXTEND:
                if (coneSensor.hasCone()) {
                    runThread(prepArmThread);
                }
                else if (gamepad.a) {
                    runThread(lowThread);
                }
                else if (gamepad.b) {
                    runThread(mediumThread);
                }
                else if (gamepad.y) {
                    runThread(highThread);
                }
                break;
            case WAIT_RETRACT:
                if (gamepad.dpad_down) {
                    slidesMotors.descendABit();
                }
                else if (gamepad.x) {
                    runThread(retractThread);
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
