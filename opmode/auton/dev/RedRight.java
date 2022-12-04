package org.firstinspires.ftc.teamcode.opmode.auton.dev;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "RedRIGHT", group = "red")
public class RedRight extends LinearOpMode {

    private static final double HEADING = Math.toRadians(270);
    private static final double WALL_POS = -1 * (70.5-(14/2.0));

    public Pose2d RIGHT_START_POSE = new Pose2d(RIGHT_CENTER_X, WALL_POS, HEADING);

    private static final double RIGHT_CENTER_X = 35;
    private static final double RIGHT_CENTER_Y = -35;
    private static final double RIGHT_LEFT_X = 12;
    private static final double RIGHT_RIGHT_X = 60;

    private static final Pose2d RIGHT_PARK_LEFT = new Pose2d(RIGHT_LEFT_X, RIGHT_CENTER_Y, HEADING);
    private static final Pose2d RIGHT_PARK_MIDDLE = new Pose2d(RIGHT_CENTER_X, RIGHT_CENTER_Y, HEADING);
    private static final Pose2d RIGHT_PARK_RIGHT = new Pose2d(RIGHT_RIGHT_X, RIGHT_CENTER_Y, HEADING);


    private SampleMecanumDrive drive;
    private Webcam webcam = new Webcam(this);
//    private RedPath redPath;

    @Override
    public void runOpMode() throws InterruptedException {
        webcam.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
//        redPath = new RedPath(drive);

        drive.setPoseEstimate(RIGHT_START_POSE);

        TrajectorySequence rightParkSequence = drive.trajectorySequenceBuilder(RIGHT_START_POSE)
                .lineToLinearHeading(RIGHT_PARK_MIDDLE)
                .waitSeconds(1)
                .lineToLinearHeading(RIGHT_PARK_RIGHT)
                .build();

        waitForStart();

//        Webcam.Side side = webcam.side();
//        webcam.stopStreaming();

        drive.followTrajectorySequenceAsync(rightParkSequence);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
