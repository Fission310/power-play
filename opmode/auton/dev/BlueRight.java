package org.firstinspires.ftc.teamcode.opmode.auton.dev;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.opmode.auton.dev.paths.BluePath;

@Autonomous (name = "BlueRIGHT", group = "blue")
public class BlueRight extends LinearOpMode {
    private SampleMecanumDrive drive;
    private Webcam webcam = new Webcam(this);
    private BluePath bluePath;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        webcam.init(hardwareMap);

        drive.setPoseEstimate(bluePath.RIGHT_START_POSE);

        bluePath = new BluePath(drive);

        waitForStart();

        Webcam.Side side = webcam.side();
        webcam.stopStreaming();

        drive.followTrajectorySequenceAsync(bluePath.rightPath(side));

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
