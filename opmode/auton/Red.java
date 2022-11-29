package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.hardware.Webcam.Side;
import org.firstinspires.ftc.teamcode.opmode.auton.paths.RedPath;

@Autonomous
public class Red extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Webcam webcam = new Webcam(this);
    private RedPath redPath;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        webcam.init(hardwareMap);

        drive.setPoseEstimate(redPath.LEFT_START_POSE);

        redPath = new RedPath(drive);

        waitForStart();

        Side side = webcam.side();
        webcam.stopStreaming();

        drive.followTrajectorySequenceAsync(redPath.leftPath(side));

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
