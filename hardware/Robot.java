package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.hardware.drivebase.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.ScoringFSM;

public class Robot extends Mechanism {

    private Drivetrain dt = new Drivetrain(opMode);
//    private Intake intake = new Intake(opMode);
    private ScoringFSM scoringFSM = new ScoringFSM(opMode);

    public Robot(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        dt.init(hwMap);
//        intake.init(hwMap);
        scoringFSM.init(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad) {
        dt.loop(gamepad);
//        intake.loop(gamepad);
        scoringFSM.loop(gamepad);
    }

//    @Override
//    public void telemetry(Telemetry telemetry) {
//        scoringFSM.telemetry(telemetry);
//    }

}
