package org.firstinspires.ftc.teamcode.opmode.auton.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BluePath {
    private SampleMecanumDrive drive;

    private static final double HEADING = Math.toRadians(90);
    private static final double WALL_POS = (70.5-(14/2.0));

    public Pose2d LEFT_START_POSE = new Pose2d(LEFT_CENTER_X, WALL_POS, HEADING);

    private static final double LEFT_CENTER_X = 35;
    private static final double LEFT_CENTER_Y = 35;
    private static final double LEFT_LEFT_X = 60;
    private static final double LEFT_RIGHT_X = 12;

    private static final Pose2d LEFT_PARK_LEFT = new Pose2d(LEFT_LEFT_X, LEFT_CENTER_Y, HEADING);
    private static final Pose2d LEFT_PARK_MIDDLE = new Pose2d(LEFT_CENTER_X, LEFT_CENTER_Y, HEADING);
    private static final Pose2d LEFT_PARK_RIGHT = new Pose2d(LEFT_RIGHT_X, LEFT_CENTER_Y, HEADING);

    // =============================================== //

    public Pose2d RIGHT_START_POSE = new Pose2d(RIGHT_CENTER_X, WALL_POS, HEADING);

    private static final double RIGHT_CENTER_X = -35;
    private static final double RIGHT_CENTER_Y = 35;
    private static final double RIGHT_LEFT_X = -12;
    private static final double RIGHT_RIGHT_X = -60;

    private static final Pose2d RIGHT_PARK_LEFT = new Pose2d(RIGHT_LEFT_X, RIGHT_CENTER_Y, HEADING);
    private static final Pose2d RIGHT_PARK_MIDDLE = new Pose2d(RIGHT_CENTER_X, RIGHT_CENTER_Y, HEADING);
    private static final Pose2d RIGHT_PARK_RIGHT = new Pose2d(RIGHT_RIGHT_X, RIGHT_CENTER_Y, HEADING);

    public BluePath(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    public TrajectorySequence leftPath(Webcam.Side side) {
        drive.setPoseEstimate(LEFT_START_POSE);

        TrajectorySequence leftParkSequence = drive.trajectorySequenceBuilder(LEFT_START_POSE)
                .lineToLinearHeading(LEFT_PARK_MIDDLE)
                .waitSeconds(1)
                .lineToLinearHeading(LEFT_PARK_LEFT)
                .build();

        TrajectorySequence middleParkSequence = drive.trajectorySequenceBuilder(LEFT_START_POSE)
                .lineToLinearHeading(LEFT_PARK_MIDDLE)
                .build();

        TrajectorySequence rightParkSequence = drive.trajectorySequenceBuilder(LEFT_START_POSE)
                .lineToLinearHeading(LEFT_PARK_MIDDLE)
                .waitSeconds(1)
                .lineToLinearHeading(LEFT_PARK_RIGHT)
                .build();

        switch (side) {
            case ONE:
                return leftParkSequence;
            case TWO:
            case NOT_FOUND:
                return middleParkSequence;
            case THREE:
                return rightParkSequence;
            default:
                return middleParkSequence;
        }
    }

    public TrajectorySequence rightPath(Webcam.Side side) {
        drive.setPoseEstimate(RIGHT_START_POSE);

        TrajectorySequence leftParkSequence = drive.trajectorySequenceBuilder(RIGHT_START_POSE)
                .lineToLinearHeading(RIGHT_PARK_MIDDLE)
                .waitSeconds(1)
                .lineToLinearHeading(RIGHT_PARK_LEFT)
                .build();

        TrajectorySequence middleParkSequence = drive.trajectorySequenceBuilder(RIGHT_START_POSE)
                .lineToLinearHeading(RIGHT_PARK_MIDDLE)
                .build();

        TrajectorySequence rightParkSequence = drive.trajectorySequenceBuilder(RIGHT_START_POSE)
                .lineToLinearHeading(RIGHT_PARK_MIDDLE)
                .waitSeconds(1)
                .lineToLinearHeading(RIGHT_PARK_RIGHT)
                .build();

        switch (side) {
            case ONE:
                return leftParkSequence;
            case TWO:
            case NOT_FOUND:
                return middleParkSequence;
            case THREE:
                return rightParkSequence;
            default:
                return middleParkSequence;
        }
    }
}
