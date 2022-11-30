package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot extends Mechanism {

    private Drivetrain dt = new Drivetrain(opMode);
    private Intake intake = new Intake(opMode);
//    private MXSensor mxSensor = new MXSensor(opMode);
    private SlidesFSM slidesFSM = new SlidesFSM(opMode);
    private ScoringFSM scoringFSM = new ScoringFSM(opMode);

    public Robot(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        dt.init(hwMap);
        intake.init(hwMap);
//        mxSensor.init(hwMap);
        slidesFSM.init(hwMap);
        scoringFSM.init(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad) {
        dt.loop(gamepad);
        intake.loop(gamepad);
//        mxSensor.loop(gamepad);
        slidesFSM.loop(gamepad);
        scoringFSM.loop(gamepad);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
//        mxSensor.telemetry(telemetry);
        slidesFSM.telemetry(telemetry);
        scoringFSM.telemetry(telemetry);
    }
}
