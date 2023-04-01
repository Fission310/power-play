package org.firstinspires.ftc.teamcode.opmode.auton.dev;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "Threaded Auto", group = "dev")
public class ThreadedAuto extends LinearOpMode {

    private volatile SampleMecanumDrive drive;
    private volatile TwoWheelTrackingLocalizer localizer;

    private static volatile Pose2d poseEstimate;
    private static volatile Pose2d poseVelocity;

    ElapsedTime time = new ElapsedTime();

    private Runnable localizerRunnable = () -> {
        for (;;) {
            try {
                localizer.update();
                poseEstimate = localizer.getPoseEstimate();
                poseVelocity = localizer.getPoseVelocity();
            } catch (IllegalThreadStateException e) {
                break;
            }
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {

         drive = new SampleMecanumDrive(hardwareMap);
         localizer = new TwoWheelTrackingLocalizer(hardwareMap, drive);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(15)
                .waitSeconds(1.5)
                .forward(15)
                .build();

        Thread localizerThread = new Thread(localizerRunnable);

        localizerThread.start();

        waitForStart();

        drive.followTrajectorySequenceAsync(sequence);

        while (opModeIsActive() && !isStopRequested()) {
            double startTime = time.milliseconds();
            drive.update(poseEstimate, poseVelocity);
            System.out.println("looptime: " + (time.milliseconds() - startTime) + " ms");
            System.out.println("\\033[2J");
        }
    }
}
