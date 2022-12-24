package org.firstinspires.ftc.teamcode.opmode.auton.dev.blue.right;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "BLUE RIGHT 5 Cone Auto", group = "_ablue")
public class FiveConeAuto extends LinearOpMode {

    private SampleMecanumDrive drive;

    private static final double HEADING = Math.toRadians(0);
    private static final double WALL_POS = 1 * (70.5 - (14 / 2.0));

    private static final double CENTER_X = -35;
    private static final Pose2d START_POSE = new Pose2d(CENTER_X, WALL_POS, HEADING);

    private static final double CONE_STACK_Y = 18;
    private static final double CONE_STACK_X = -60;

    private static final double HIGH_GOAL_X = -22.75;
    private static final double HIGH_GOAL_Y = 6;
    private static final double PRELOAD_X_OFFSET = -0.5;
    private static final double PRELOAD_CONE_STACK_Y = 16;

    private static final double HIGH_GOAL_ANGLE = 140 - 180;
    private static final double HIGH_GOAL_TANGENT = 180 - 180;
    private static final double CONE_STACK_ANGLE = 320 - 180;
    private static final double CONE_STACK_END_ANGLE = 0 + 180;
    private static final double CONE_STACK_ANGLE_OFFSET = 20;

    private static final double PARK_LEFT_X = -10;

    private static final Pose2d PARK_LEFT = new Pose2d(PARK_LEFT_X, CONE_STACK_Y, HEADING);
    private static final Pose2d PARK_MIDDLE = new Pose2d(CENTER_X, CONE_STACK_Y, HEADING);

    private static final Vector2d HIGH_GOAL_VECTOR = new Vector2d(HIGH_GOAL_X, HIGH_GOAL_Y);
    private static final Vector2d CONE_STACK = new Vector2d(CONE_STACK_X, CONE_STACK_Y);

    private enum TrajectoryState {
        PRELOAD,
        PRELOAD_TO_CS,
        CS_TO_HG,
        HG_TO_CS,
        PARK,
        IDLE
    }
    TrajectoryState trajectoryState = TrajectoryState.PRELOAD;

    /** VERY IMPORTANT **/
    private static final int CONE_COUNT = 5;
    private static int conesScored = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(START_POSE);

        TrajectorySequence preload = drive.trajectorySequenceBuilder(START_POSE)
                .setTangent(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(CENTER_X, HIGH_GOAL_Y, HEADING))
                .lineToLinearHeading(new Pose2d(HIGH_GOAL_X + PRELOAD_X_OFFSET, HIGH_GOAL_Y, HEADING))
                .build();

        TrajectorySequence preloadToConeStack = drive.trajectorySequenceBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(CENTER_X, HIGH_GOAL_Y, HEADING))
                .lineToLinearHeading(new Pose2d(CENTER_X, PRELOAD_CONE_STACK_Y, HEADING))
                .setReversed(true)
                .setTangent(Math.toRadians(CONE_STACK_ANGLE + CONE_STACK_ANGLE_OFFSET))
                .splineToConstantHeading(CONE_STACK, Math.toRadians(CONE_STACK_END_ANGLE))
                .build();

        TrajectorySequence coneStackToHighGoal = drive.trajectorySequenceBuilder(preloadToConeStack.end())
                .setReversed(false)
                .setTangent(Math.toRadians(HIGH_GOAL_TANGENT))
                .splineTo(HIGH_GOAL_VECTOR, Math.toRadians(HIGH_GOAL_ANGLE))
//                .waitSeconds(4)
                .build();

        TrajectorySequence highGoalToConeStack = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setReversed(true)
                .setTangent(Math.toRadians(CONE_STACK_ANGLE))
                .splineTo(CONE_STACK, Math.toRadians(CONE_STACK_END_ANGLE))
                .build();

        TrajectorySequence toLeftPark = drive.trajectorySequenceBuilder(highGoalToConeStack.end())
                .lineToLinearHeading(PARK_LEFT)
                .build();

        TrajectorySequence toMiddlePark = drive.trajectorySequenceBuilder(highGoalToConeStack.end())
                .lineToLinearHeading(PARK_MIDDLE)
                .build();

        waitForStart();

        // drive to preload
        drive.followTrajectorySequenceAsync(preload);

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("cones scored", conesScored);
            telemetry.addData("current state", trajectoryState);
            telemetry.update();
            drive.update();

            switch (trajectoryState) {
                case PRELOAD:
                    if (!drive.isBusy()) {
                        trajectoryState = TrajectoryState.PRELOAD_TO_CS;
                        drive.followTrajectorySequenceAsync(preloadToConeStack);
                        conesScored += 1;
                    }
                    break;
                case PRELOAD_TO_CS:
                    if (!drive.isBusy()) {
                        trajectoryState = TrajectoryState.CS_TO_HG;
                        drive.followTrajectorySequenceAsync(coneStackToHighGoal);
                    }
                    break;
                case CS_TO_HG:
                    if (!drive.isBusy()) {
                        trajectoryState = TrajectoryState.HG_TO_CS;
                        drive.followTrajectorySequenceAsync(highGoalToConeStack);
                        conesScored += 1;
                    }
                    break;
                case HG_TO_CS:
                    if (conesScored == CONE_COUNT) {
                        trajectoryState = TrajectoryState.PARK;
                    } else {
                        if (!drive.isBusy()) {
                            trajectoryState = TrajectoryState.CS_TO_HG;
                            drive.followTrajectorySequenceAsync(coneStackToHighGoal);
                        }
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                        trajectoryState = TrajectoryState.IDLE;
                        drive.followTrajectorySequenceAsync(toLeftPark);
                    }
                    // switch webcam
                    break;
                case IDLE:
                    break;
            }
        }
    }
}
