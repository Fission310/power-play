package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

public class DriftCompensatedRobot extends Mechanism {

    private DriftCompensatedDrivetrain dt = new DriftCompensatedDrivetrain(opMode);
    private ScoringFSM scoringFSM = new ScoringFSM(opMode);

    public DriftCompensatedRobot(LinearOpMode opMode) { this.opMode = opMode; }

    public void init(HardwareMap hwMap) {
        dt.init(hwMap);
        scoringFSM.init(hwMap);
    }

    public void loop(Gamepad gamepad) {
        dt.loop(gamepad);
        scoringFSM.loop(gamepad);
    }
}
