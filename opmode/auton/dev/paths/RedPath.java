package org.firstinspires.ftc.teamcode.opmode.auton.dev.paths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.SignalSleeveWebcam.Side;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
public class RedPath {

    private SampleMecanumDrive drive;

    private static final double HEADING = Math.toRadians(180);
    private static final double WALL_POS = -1 * (70.5-(14/2.0));

    private static final double RIGHT_CENTER_X = 35;

    private static final double RIGHT_HIGH_GOAL_Y = 0;
    private static final double RIGHT_HIGH_GOAL_SCORE_X = 28;

    private static final Pose2d RIGHT_START_POSE = new Pose2d(RIGHT_CENTER_X, WALL_POS, HEADING);

    public RedPath(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    public TrajectorySequence rightPath(Side side) {
        drive.setPoseEstimate(RIGHT_START_POSE);

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(RIGHT_START_POSE)
                .lineToLinearHeading(new Pose2d(RIGHT_CENTER_X, RIGHT_HIGH_GOAL_Y, HEADING))
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(RIGHT_HIGH_GOAL_SCORE_X, RIGHT_HIGH_GOAL_Y, HEADING))
                .waitSeconds(2)
                .build();

        return sequence;
    }

}
