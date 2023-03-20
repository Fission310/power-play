package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.CommandBasedRobot;
import org.firstinspires.ftc.teamcode.hardware.DriftCompensatedRobot;

@TeleOp (name = "Command Main", group = "_amain")
public class CommandBasedMain extends LinearOpMode {

    private CommandBasedRobot robot = new CommandBasedRobot(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            robot.loop(gamepad1);

            robot.telemetry(telemetry);
        }
    }
}
