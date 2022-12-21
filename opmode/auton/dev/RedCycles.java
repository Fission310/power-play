package org.firstinspires.ftc.teamcode.opmode.auton.dev;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.QuinticSpline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "Red Cycles", group = "_main")
public class RedCycles extends LinearOpMode {

    private SampleMecanumDrive drive;

//    private static final double HEADING = Math.toRadians(180);
//    private static final double WALL_POS = -1 * (70.5-(14/2.0));
//
//    private static final double RIGHT_CENTER_X = 35;
//
//    private static final double RIGHT_HIGH_GOAL_Y = 0;
//    private static final double RIGHT_HIGH_GOAL_SCORE_X = 28;
//
//    private static final double RIGHT_CONE_STACK_Y = -13;
//    private static final double RIGHT_CONE_STACK_X = 62;
//
//    private static final Pose2d RIGHT_START_POSE = new Pose2d(RIGHT_CENTER_X, WALL_POS, HEADING);

    // FOR SPLINE CYCLE //
    private static final double HEADING = Math.toRadians(180);
    private static final double WALL_POS = -1 * (70.5-(14/2.0));

    private static final double RIGHT_CENTER_X = 35;
    private static final Pose2d RIGHT_START_POSE = new Pose2d(RIGHT_CENTER_X, WALL_POS, HEADING);

    private static final double RIGHT_CONE_STACK_Y = -17;
    private static final double RIGHT_CONE_STACK_X = 60;

    private static final double RIGHT_HIGH_GOAL_X = 24;
    private static final double RIGHT_HIGH_GOAL_Y = -8;

    private static double HIGH_GOAL_ANGLE = 140;
    private static double CONE_STACK_ANGLE = 320;

    private static final Pose2d HIGH_GOAL_POSE = new Pose2d(RIGHT_HIGH_GOAL_X, RIGHT_HIGH_GOAL_Y, Math.toRadians(140));
    private static final Vector2d HIGH_GOAL_VECTOR = new Vector2d(RIGHT_HIGH_GOAL_X, RIGHT_HIGH_GOAL_Y);
    private static final Vector2d CONE_STACK = new Vector2d(RIGHT_CONE_STACK_X, RIGHT_CONE_STACK_Y);

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(RIGHT_START_POSE);

//        TrajectorySequence cycle = drive.trajectorySequenceBuilder(drive.getPoseEstimate())


        TrajectorySequence splineCycle = drive.trajectorySequenceBuilder(RIGHT_START_POSE)
                // drive to high goal
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(HIGH_GOAL_POSE, Math.toRadians(130))
                // drive to high goal

                // score
                .waitSeconds(0.5)
                // score

                // ======================= //

                // start cycle

                // go to cone stack
                .setReversed(true)
                .setTangent(Math.toRadians(CONE_STACK_ANGLE))
                .splineTo(CONE_STACK, Math.toRadians(0))

                .waitSeconds(0.5)

                // go to score
                .setReversed(false)
                .setTangent(Math.toRadians(180))
                .splineTo(HIGH_GOAL_VECTOR, Math.toRadians(HIGH_GOAL_ANGLE))

                .waitSeconds(0.5)

                // end cycle

                // ======================= //

                // start cycle

                // go to cone stack
                .setReversed(true)
                .setTangent(Math.toRadians(CONE_STACK_ANGLE))
                .splineTo(CONE_STACK, Math.toRadians(0))

                .waitSeconds(0.5)

                // go to score
                .setReversed(false)
                .setTangent(Math.toRadians(180))
                .splineTo(HIGH_GOAL_VECTOR, Math.toRadians(HIGH_GOAL_ANGLE))

                .waitSeconds(0.5)

                // end cycle

                // ======================= //

                // start cycle

                // go to cone stack
                .setReversed(true)
                .setTangent(Math.toRadians(CONE_STACK_ANGLE))
                .splineTo(CONE_STACK, Math.toRadians(0))

                .waitSeconds(0.5)

                // go to score
                .setReversed(false)
                .setTangent(Math.toRadians(180))
                .splineTo(HIGH_GOAL_VECTOR, Math.toRadians(HIGH_GOAL_ANGLE))

                .waitSeconds(0.5)

                // end cycle

                // ======================= //

                // start cycle

                // go to cone stack
                .setReversed(true)
                .setTangent(Math.toRadians(CONE_STACK_ANGLE))
                .splineTo(CONE_STACK, Math.toRadians(0))

                .waitSeconds(0.5)

                // go to score
                .setReversed(false)
                .setTangent(Math.toRadians(180))
                .splineTo(HIGH_GOAL_VECTOR, Math.toRadians(HIGH_GOAL_ANGLE))

                .waitSeconds(0.5)

                // end cycle

                // ======================= //

                // start cycle

                // go to cone stack
                .setReversed(true)
                .setTangent(Math.toRadians(CONE_STACK_ANGLE))
                .splineTo(CONE_STACK, Math.toRadians(0))

                .waitSeconds(0.5)

                // go to score
                .setReversed(false)
                .setTangent(Math.toRadians(180))
                .splineTo(HIGH_GOAL_VECTOR, Math.toRadians(HIGH_GOAL_ANGLE))

                .waitSeconds(0.5)

                // end cycle

                .build();

        waitForStart();

        drive.followTrajectorySequenceAsync(splineCycle);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }

}
