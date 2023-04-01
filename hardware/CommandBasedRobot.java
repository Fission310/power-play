package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.hardware.drivebase.DriftCompensatedDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.ScoringCommandBased;

public class CommandBasedRobot extends Mechanism {
    private DriftCompensatedDrivetrain dt = new DriftCompensatedDrivetrain(opMode);
    private ScoringCommandBased scoring = new ScoringCommandBased(opMode);

    public CommandBasedRobot(LinearOpMode opMode) { this.opMode = opMode; }

    public void init(HardwareMap hwMap) {
        dt.init(hwMap);
        scoring.init(hwMap);
    }

    public void loop(Gamepad gamepad) {
        dt.loop(gamepad);
        scoring.loop(gamepad);
    }
}
