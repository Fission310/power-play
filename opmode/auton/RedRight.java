package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.opmode.auton.paths.RedPath;

@Autonomous (name = "RedRIGHT", group = "_red")
public class RedRight extends LinearOpMode {
    private SampleMecanumDrive drive;
    private Webcam webcam = new Webcam(this);
    private RedPath redPath;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        webcam.init(hardwareMap);

        drive.setPoseEstimate(redPath.RIGHT_START_POSE);

        redPath = new RedPath(drive);

        waitForStart();

        Webcam.Side side = webcam.side();
        webcam.stopStreaming();

        drive.followTrajectorySequenceAsync(redPath.rightPath(side));

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
