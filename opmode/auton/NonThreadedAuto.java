package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "NON Threaded Auto", group = "dev")
public class NonThreadedAuto extends LinearOpMode {

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//    TwoWheelTrackingLocalizer localizer = new TwoWheelTrackingLocalizer(hardwareMap, drive);

//    private static Pose2d poseEstimate;
//    private static Pose2d poseVelocity;

    ElapsedTime time = new ElapsedTime();

//    private Runnable localizerRunnable = () -> {
//        for (;;) {
//            try {
//                localizer.update();
//                poseEstimate = localizer.getPoseEstimate();
//                poseVelocity = localizer.getPoseVelocity();
//            } catch (IllegalThreadStateException e) {
//                break;
//            }
//        }
//    };

    @Override
    public void runOpMode() throws InterruptedException {

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(15)
                .waitSeconds(1.5)
                .forward(15)
                .build();

//        Thread localizerThread = new Thread(localizerRunnable);
//        localizerThread.start();

        waitForStart();

        drive.followTrajectorySequenceAsync(sequence);

        while (opModeIsActive() && !isStopRequested()) {
            double startTime = time.milliseconds();
            drive.update();
            System.out.println("looptime: " + (time.milliseconds() - startTime) + " ms");
            System.out.println("\\033[2J");
        }
    }
}
