package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlidesFSM extends Mechanism {

    SlidesMotors slidesMotors = new SlidesMotors(opMode);

    public SlidesFSM(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        slidesMotors.init(hwMap);
    }

    public enum SlidesState {
        REST,
        WAIT_EXTEND,
        WAIT_RETRACT
    }
    SlidesState slidesState;

    public Runnable low = () -> {
        try {
            slidesMotors.extendLow();
            Thread.sleep(1000);
            // TODO: Thread.sleep to allow mechanisms to work
            slidesState = SlidesState.WAIT_RETRACT;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable medium = () -> {
      try {
          slidesMotors.extendMedium();
          Thread.sleep(1000);
          slidesState = SlidesState.WAIT_RETRACT;
      } catch (InterruptedException e) {
          e.printStackTrace();
      }
    };

    public Runnable high = () -> {
      try {
          slidesMotors.extendHigh();
          Thread.sleep(1000);
          slidesState = SlidesState.WAIT_RETRACT;
      } catch (InterruptedException e) {
          e.printStackTrace();
      }
    };

    @Override
    public void loop(Gamepad gamepad) {
        switch (slidesState) {
            case REST:
                slidesMotors.rest();
                slidesState = SlidesState.WAIT_EXTEND;
                break;
            case WAIT_EXTEND:
                if (gamepad.a) {
                    low.run();
                }
                if (gamepad.b) {
                    medium.run();
                }
                if (gamepad.y) {
                    high.run();
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
