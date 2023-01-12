package org.firstinspires.ftc.teamcode.opmode.auton.blue.right;

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

@Autonomous (name = "BLUE RIGHT 5 Cone Auto", group = "_ared")
public class FiveConeAuto extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Clamp clamp;
    private Arm arm;
    private SlidesMotors slides;
    private SignalSleeveWebcam signalSleeveWebcam = new SignalSleeveWebcam(this, "SignalSleeveWebcam");

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
            arm.moveToPos(AutoConstants.ARM_CONE_STACK_POSITIONS[conesScored]); // autoIntakePos
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
    private static final int CONE_COUNT = 4;
    private static int conesScored = 0;

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

        drive.setPoseEstimate(AutoConstants.BR_START_POSE);

        TrajectorySequence preload = drive.trajectorySequenceBuilder(AutoConstants.BR_START_POSE)
                .setTangent(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(AutoConstants.BR_CENTER_X, AutoConstants.BR_HIGH_GOAL_Y, AutoConstants.BR_HEADING))
                .lineToLinearHeading(new Pose2d(AutoConstants.BR_HIGH_GOAL_X + AutoConstants.BR_PRELOAD_X_OFFSET, AutoConstants.BR_HIGH_GOAL_Y, AutoConstants.BR_HEADING))
                .waitSeconds(0.2)
                .build();

        preloadToConeStack = drive.trajectorySequenceBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(AutoConstants.BR_CENTER_X, AutoConstants.BR_HIGH_GOAL_Y, AutoConstants.BR_HEADING))
                .lineToLinearHeading(new Pose2d(AutoConstants.BR_CENTER_X, AutoConstants.BR_PRELOAD_CONE_STACK_Y, AutoConstants.BR_HEADING))
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.BR_CONE_STACK_ANGLE + AutoConstants.BR_CONE_STACK_ANGLE_OFFSET))
                .splineToConstantHeading(AutoConstants.BR_CONE_STACK_VECTOR, Math.toRadians(AutoConstants.BR_CONE_STACK_END_ANGLE))
                .build();

        coneStackToHighGoal = drive.trajectorySequenceBuilder(preloadToConeStack.end())
                .setReversed(false)
                .setTangent(Math.toRadians(AutoConstants.BR_HIGH_GOAL_TANGENT))
                .splineTo(AutoConstants.BR_HIGH_GOAL_VECTOR, Math.toRadians(AutoConstants.BR_HIGH_GOAL_ANGLE))
                .waitSeconds(0.2)
                .build();

        highGoalToConeStack = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.BR_CONE_STACK_ANGLE))
                .splineTo(AutoConstants.BR_CONE_STACK_VECTOR, Math.toRadians(AutoConstants.BR_CONE_STACK_END_ANGLE))
                .build();

        TrajectorySequence toLeftPark = drive.trajectorySequenceBuilder(highGoalToConeStack.end())
                .lineToLinearHeading(AutoConstants.BR_PARK_LEFT)
                .build();

        TrajectorySequence toMiddlePark = drive.trajectorySequenceBuilder(highGoalToConeStack.end())
                .lineToLinearHeading(AutoConstants.BR_PARK_MIDDLE)
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
                            drive.followTrajectorySequenceAsync(preloadToConeStack);
                            trajectoryState = TrajectoryState.PRELOAD_TO_CS;
                            time.reset();
                            canSlidesExtend = true;
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
                            drive.followTrajectorySequenceAsync(coneStackToHighGoal);
                            trajectoryState = TrajectoryState.CS_TO_HG;
                            canSlidesExtend = true;
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
                            drive.followTrajectorySequenceAsync(highGoalToConeStack);
                            trajectoryState = TrajectoryState.HG_TO_CS;
                            canSlidesExtend = true;
                            time.reset();
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
                                drive.followTrajectorySequenceAsync(coneStackToHighGoal);
                                trajectoryState = TrajectoryState.CS_TO_HG;
                                canSlidesExtend = true;
                            }
                        }
                    }
                    break;
                case PARK:
                    switch (parkSide) {
                        case ONE:
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
                        case THREE:
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
