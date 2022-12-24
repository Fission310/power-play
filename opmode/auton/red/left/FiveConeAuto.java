package org.firstinspires.ftc.teamcode.opmode.auton.red.left;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.opmode.auton.AutoConstants;

@Autonomous (name = "RED LEFT 5 Cone Auto", group = "_ared")
public class FiveConeAuto extends LinearOpMode {

    private SampleMecanumDrive drive;

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

        drive.setPoseEstimate(AutoConstants.RL_START_POSE);

        TrajectorySequence preload = drive.trajectorySequenceBuilder(AutoConstants.RL_START_POSE)
                .setTangent(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(AutoConstants.RL_CENTER_X, AutoConstants.RL_HIGH_GOAL_Y, AutoConstants.RL_HEADING))
                .lineToLinearHeading(new Pose2d(AutoConstants.RL_HIGH_GOAL_X + AutoConstants.RL_PRELOAD_X_OFFSET, AutoConstants.RL_HIGH_GOAL_Y, AutoConstants.RL_HEADING))
                .build();

        TrajectorySequence preloadToConeStack = drive.trajectorySequenceBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(AutoConstants.RL_CENTER_X, AutoConstants.RL_HIGH_GOAL_Y, AutoConstants.RL_HEADING))
                .lineToLinearHeading(new Pose2d(AutoConstants.RL_CENTER_X, AutoConstants.RL_PRELOAD_CONE_STACK_Y, AutoConstants.RL_HEADING))
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.RL_CONE_STACK_ANGLE + AutoConstants.RL_CONE_STACK_ANGLE_OFFSET))
                .splineToConstantHeading(AutoConstants.RL_CONE_STACK_VECTOR, Math.toRadians(AutoConstants.RL_CONE_STACK_END_ANGLE))
                .build();

        TrajectorySequence coneStackToHighGoal = drive.trajectorySequenceBuilder(preloadToConeStack.end())
                .setReversed(false)
                .setTangent(Math.toRadians(AutoConstants.RL_HIGH_GOAL_TANGENT))
                .splineTo(AutoConstants.RL_HIGH_GOAL_VECTOR, Math.toRadians(AutoConstants.RL_HIGH_GOAL_ANGLE))
//                .waitSeconds(4)
                .build();

        TrajectorySequence highGoalToConeStack = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.RL_CONE_STACK_ANGLE))
                .splineTo(AutoConstants.RL_CONE_STACK_VECTOR, Math.toRadians(AutoConstants.RL_CONE_STACK_END_ANGLE))
                .build();

        TrajectorySequence toLeftPark = drive.trajectorySequenceBuilder(highGoalToConeStack.end())
                .lineToLinearHeading(AutoConstants.RL_PARK_LEFT)
                .build();

        TrajectorySequence toMiddlePark = drive.trajectorySequenceBuilder(highGoalToConeStack.end())
                .lineToLinearHeading(AutoConstants.RL_PARK_MIDDLE)
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
