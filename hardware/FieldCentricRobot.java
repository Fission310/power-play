package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.hardware.drivebase.FieldCentricDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.ScoringFSM;

public class FieldCentricRobot extends Mechanism {

    private FieldCentricDrivetrain dt = new FieldCentricDrivetrain(opMode);
    private ScoringFSM scoringFSM = new ScoringFSM(opMode);

    public FieldCentricRobot(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        dt.init(hwMap);
        scoringFSM.init(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad) {
        dt.loop(gamepad);
        scoringFSM.loop(gamepad);
    }
}
