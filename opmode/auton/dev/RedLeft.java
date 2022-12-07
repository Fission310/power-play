package org.firstinspires.ftc.teamcode.opmode.auton.dev;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.SignalSleeveWebcam;
import org.firstinspires.ftc.teamcode.hardware.SignalSleeveWebcam.Side;
import org.firstinspires.ftc.teamcode.opmode.auton.dev.paths.RedPath;

@Autonomous (name = "RedLEFT", group = "red")
public class RedLeft extends LinearOpMode {

    private SampleMecanumDrive drive;
    private SignalSleeveWebcam signalSleeveWebcam = new SignalSleeveWebcam(this);
    private RedPath redPath;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        signalSleeveWebcam.init(hardwareMap);

        drive.setPoseEstimate(redPath.LEFT_START_POSE);

        redPath = new RedPath(drive);

        waitForStart();

        Side side = signalSleeveWebcam.side();
        signalSleeveWebcam.stopStreaming();

        drive.followTrajectorySequenceAsync(redPath.leftPath(side));

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
