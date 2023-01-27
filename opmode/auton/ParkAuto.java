package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.SignalSleeveWebcam;
import org.firstinspires.ftc.teamcode.opmode.auton.AutoConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "Red Park Auto", group = "_ared")
public class ParkAuto extends LinearOpMode {

    private SampleMecanumDrive drive;
    private SignalSleeveWebcam signalSleeveWebcam = new SignalSleeveWebcam(this, "rightWebcam", SignalSleeveWebcam.ROBOT_SIDE.CONTROL_HUB);

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        signalSleeveWebcam.init(hardwareMap);

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(AutoConstants.FORWARD_DIST)
                .waitSeconds(0.5)
                .forward(AutoConstants.LATERAL_DIST)
                .build();

        TrajectorySequence middlePark = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(AutoConstants.FORWARD_DIST)
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(AutoConstants.FORWARD_DIST)
                .waitSeconds(0.5)
                .back(AutoConstants.LATERAL_DIST)
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

        signalSleeveWebcam.stopStreaming();

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
