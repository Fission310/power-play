package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.SignalSleeveWebcam;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "ALL AUTO", group = "_main")
public class AllAuto extends LinearOpMode {

    public static double FORWARD_DIST = 36;
    public static double LATERAL_DIST = 23;

    private SampleMecanumDrive drive;
    private SignalSleeveWebcam signalSleeveWebcam = new SignalSleeveWebcam(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        signalSleeveWebcam.init(hardwareMap);

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(FORWARD_DIST)
                .waitSeconds(0.5)
                .strafeLeft(LATERAL_DIST)
                .build();

        TrajectorySequence middlePark = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(FORWARD_DIST)
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(FORWARD_DIST)
                .waitSeconds(0.5)
                .strafeRight(LATERAL_DIST)
                .build();

        waitForStart();

        switch (signalSleeveWebcam.side()) {
            case ONE:
                drive.followTrajectorySequenceAsync(leftPark);
                break;
            case TWO:
                drive.followTrajectorySequenceAsync(middlePark);
                break;
            case THREE:
                drive.followTrajectorySequenceAsync(rightPark);
                break;
            case NOT_FOUND:
            default:
                drive.followTrajectorySequenceAsync(middlePark);
                break;
        }

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }

    }
}
