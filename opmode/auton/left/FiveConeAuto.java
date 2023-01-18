package org.firstinspires.ftc.teamcode.opmode.auton.left;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Clamp;
import org.firstinspires.ftc.teamcode.hardware.SignalSleeveWebcam;
import org.firstinspires.ftc.teamcode.hardware.SlidesMotors;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.opmode.auton.AutoConstants;

@Autonomous (name = "LEFT_SIDE 5 Cone Auto", group = "_ared")
public class FiveConeAuto extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Clamp clamp;
    private Arm arm;
    private SlidesMotors slides;
    private SignalSleeveWebcam signalSleeveWebcam = new SignalSleeveWebcam(this, "rightWebcam");

    private SignalSleeveWebcam.Side parkSide = SignalSleeveWebcam.Side.ONE;

    private Thread scoreReadyThread;
    private Thread scoreThread;

    ElapsedTime time = new ElapsedTime();

    private TrajectorySequence preloadToConeStack;
    private TrajectorySequence coneStackToHighGoal;
    private TrajectorySequence highGoalToConeStack;

    private boolean canContinue = false;
    private boolean canSlidesExtend = false;

    public Runnable scoreReady = () -> {
        try {
            // extend slides lvl 3
            // rotate arm to intake pos
            slides.extendHigh();
            Thread.sleep(100);
            arm.autoScorePos();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable score = () -> {
        try {
            Thread.sleep(200);
            arm.moveToPos(AutoConstants.ARM_CONE_STACK_POSITIONS[conesScored]);
            canContinue = true;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public void runThread(Thread thread) {
        try {
            thread.start();
        } catch (IllegalThreadStateException ignored) {}
    }

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
    private static final int CONE_COUNT = 2;
    private static int conesScored;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        clamp = new Clamp(this);
        arm = new Arm(this);
        slides = new SlidesMotors(this);
        signalSleeveWebcam.init(hardwareMap);

        clamp.init(hardwareMap);
        arm.init(hardwareMap);
        slides.init(hardwareMap);

        scoreReadyThread = new Thread(scoreReady);
        scoreThread = new Thread(score);

        drive.setPoseEstimate(AutoConstants.RL_START_POSE);

        conesScored = 0;

        TrajectorySequence preload = drive.trajectorySequenceBuilder(AutoConstants.RL_START_POSE)
                .setTangent(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(AutoConstants.RL_CENTER_X, AutoConstants.RL_HIGH_GOAL_Y - AutoConstants.RL_HIGH_GOAL_Y_PRELOAD_OFFSET, AutoConstants.RL_HEADING))
                .lineToLinearHeading(new Pose2d(AutoConstants.RL_HIGH_GOAL_X + AutoConstants.RL_PRELOAD_X_OFFSET, AutoConstants.RL_HIGH_GOAL_Y, AutoConstants.RL_HEADING))
                .waitSeconds(0.2)
                .build();

        preloadToConeStack = drive.trajectorySequenceBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(AutoConstants.RL_CENTER_X, AutoConstants.RL_HIGH_GOAL_Y, AutoConstants.RL_HEADING))
                .lineToLinearHeading(new Pose2d(AutoConstants.RL_CENTER_X, AutoConstants.RL_PRELOAD_CONE_STACK_Y + 1, AutoConstants.RL_HEADING))
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.RL_CONE_STACK_ANGLE + AutoConstants.RL_CONE_STACK_ANGLE_OFFSET))
                .splineToConstantHeading(AutoConstants.RL_PRELOAD_CONE_STACK_VECTOR, Math.toRadians(AutoConstants.RL_CONE_STACK_END_ANGLE))
                .build();

        coneStackToHighGoal = drive.trajectorySequenceBuilder(preloadToConeStack.end())
                .setReversed(false)
                .setTangent(Math.toRadians(AutoConstants.RL_HIGH_GOAL_TANGENT))
                .splineTo(AutoConstants.RL_HIGH_GOAL_VECTOR, Math.toRadians(AutoConstants.RL_HIGH_GOAL_ANGLE))
                .waitSeconds(0.35)
                .build();

        highGoalToConeStack = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
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

        clamp.close();
        arm.intakePos();

        waitForStart();

        parkSide = signalSleeveWebcam.side();

        signalSleeveWebcam.stopStreaming();

        // drive to preload
        drive.followTrajectorySequenceAsync(preload);
        runThread(scoreReadyThread);

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("cones scored", conesScored);
            telemetry.addData("current state", trajectoryState);
            telemetry.addData("canContinue", canContinue);
            telemetry.addData("canSlidesExtend", canSlidesExtend);
            telemetry.addData("time", time);
            telemetry.update();
            drive.update();
            slides.update();

            /**  TODO:
             *
             *   get rid of canContinue, use timers
             *
             */

            switch (trajectoryState) {
                case PRELOAD:
                    if (!drive.isBusy()) {
                        clamp.open();
                        runThread(scoreThread);
                        if (canContinue) {
                            conesScored += 1;
                            slides.extendToPosition(AutoConstants.SLIDE_EXTEND_POSITIONS[conesScored]); // (8.2)
                            canContinue = false;
                            canSlidesExtend = true;
                            time.reset();
                            drive.followTrajectorySequenceAsync(preloadToConeStack);
                            trajectoryState = TrajectoryState.PRELOAD_TO_CS;
                        }
                    }
                    break;
                case PRELOAD_TO_CS:
                    if (!drive.isBusy()) {
                        clamp.close();
                        if (canSlidesExtend) {
                            slides.extendHigh();
                            canSlidesExtend = false;
                        }
                        if (time.seconds() >= AutoConstants.DELAY_PRELOAD_PICKUP) {
                            arm.autoScorePos();
                            canContinue = false;
                            canSlidesExtend = true;
                            drive.followTrajectorySequenceAsync(coneStackToHighGoal);
                            trajectoryState = TrajectoryState.CS_TO_HG;
                        }
                    }
                    break;
                case CS_TO_HG:
                    if (!drive.isBusy()) {
                        clamp.open();
                        runThread(scoreThread);
                        if (canContinue) {
                            conesScored += 1;
                            slides.extendToPosition(AutoConstants.SLIDE_EXTEND_POSITIONS[conesScored]);
                            canContinue = false;
                            canSlidesExtend = true;
                            time.reset();
                            drive.followTrajectorySequenceAsync(highGoalToConeStack);
                            trajectoryState = TrajectoryState.HG_TO_CS;
                        }
                    }
                    break;
                case HG_TO_CS:
                    if (conesScored >= CONE_COUNT) {
                        slides.rest();
                        arm.intakePos();
                        clamp.open();
                        trajectoryState = TrajectoryState.PARK;
                    } else {
                        if (!drive.isBusy()) {
                            clamp.close();
                            if (canSlidesExtend) {
                                slides.extendHigh();
                                canSlidesExtend = false;
                            }
                            if (time.seconds() >= AutoConstants.DELAY_PICKUP) {
                                arm.autoScorePos();
                                canContinue = false;
                                canSlidesExtend = true;
                                drive.followTrajectorySequenceAsync(coneStackToHighGoal);
                                trajectoryState = TrajectoryState.CS_TO_HG;
                            }
                        }
                    }
                    break;
                case PARK:
                    arm.intakePos();
                    clamp.open();
                    switch (parkSide) {
                        case THREE:
                            if (!drive.isBusy()) {
                                trajectoryState = TrajectoryState.IDLE;
                            }
                            break;
                        case TWO:
                            if (!drive.isBusy()) {
                                trajectoryState = TrajectoryState.IDLE;
                                drive.followTrajectorySequenceAsync(toMiddlePark);
                            }
                            break;
                        case ONE:
                            if (!drive.isBusy()) {
                                trajectoryState = TrajectoryState.IDLE;
                                drive.followTrajectorySequenceAsync(toLeftPark);
                            }
                            break;
                        case NOT_FOUND:
                        default:
                            if (!drive.isBusy()) {
                                trajectoryState = TrajectoryState.IDLE;
                            }
                            break;
                    }
                    break;
                case IDLE:
                    arm.intakePos();
                    clamp.open();
                    break;
            }
        }
    }
}
