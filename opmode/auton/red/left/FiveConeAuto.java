package org.firstinspires.ftc.teamcode.opmode.auton.red.left;

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

@Autonomous (name = "RED LEFT 5 Cone Auto", group = "_ared")
public class FiveConeAuto extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Clamp clamp;
    private Arm arm;
    private SlidesMotors slides;
    private SignalSleeveWebcam signalSleeveWebcam = new SignalSleeveWebcam(this, "SignalSleeveWebcam");

    private SignalSleeveWebcam.Side parkSide;

    private Thread scoreReadyThread;
    private Thread scoreThread;

    private Thread pickupReadyThread;
    private Thread pickupThread;

    ElapsedTime time = new ElapsedTime();

    public Runnable scoreReady = () -> {
        try {
            // extend slides lvl 3
            // rotate arm to intake pos
            slides.extendHigh();
            Thread.sleep(100);
            arm.intakePos();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable score = () -> {
        try {
            clamp.open();
            Thread.sleep(100);
            arm.scorePos();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable pickupReady = () -> {
        try {
            // open clamp
            // rotate arm to score pos (back intake)
            // retract slides to cone stack pos
            clamp.open();
            Thread.sleep(100);
            arm.scorePos();
            Thread.sleep(500);
            slides.extendConeStack();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable pickup = () -> {
        try {
            clamp.close();
            Thread.sleep(100);
            slides.extendHigh();
            Thread.sleep(100);
            arm.intakePos();
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
    private static final int CONE_COUNT = 5;
    private static int conesScored = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        clamp = new Clamp(this);
        arm = new Arm(this);
        slides = new SlidesMotors(this);
        signalSleeveWebcam.init(hardwareMap);

        parkSide = signalSleeveWebcam.side();

        clamp.init(hardwareMap);
        arm.init(hardwareMap);
        slides.init(hardwareMap);

        scoreReadyThread = new Thread(scoreReady);
        scoreThread = new Thread(score);

        pickupReadyThread = new Thread(pickupReady);
        pickupThread = new Thread(pickup);

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

        clamp.close();
        arm.intakePos();

        waitForStart();

        signalSleeveWebcam.stopStreaming();

        // drive to preload
        drive.followTrajectorySequenceAsync(preload);

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("cones scored", conesScored);
            telemetry.addData("current state", trajectoryState);
            telemetry.update();
            drive.update();
            slides.update();

            switch (trajectoryState) {
                case PRELOAD:
                    runThread(scoreReadyThread);
                    time.reset();
                    if (!drive.isBusy()) {
                        runThread(scoreThread);
                        if (time.seconds() >= AutoConstants.DELAY_SCORE) {
                            trajectoryState = TrajectoryState.PRELOAD_TO_CS;
                            drive.followTrajectorySequenceAsync(preloadToConeStack);
                            conesScored += 1;
                        }
                    }
                    break;
                case PRELOAD_TO_CS:
                    runThread(pickupReadyThread);
                    time.reset();
                    if (!drive.isBusy()) {
                        runThread(pickupThread);
                        if (time.seconds() >= AutoConstants.DELAY_PICKUP) {
                            trajectoryState = TrajectoryState.CS_TO_HG;
                            drive.followTrajectorySequenceAsync(coneStackToHighGoal);
                        }
                    }
                    break;
                case CS_TO_HG:
                    runThread(scoreReadyThread);
                    if (!drive.isBusy()) {
                        runThread(scoreThread);
                        if (time.seconds() >= AutoConstants.DELAY_SCORE) {
                            trajectoryState = TrajectoryState.HG_TO_CS;
                            drive.followTrajectorySequenceAsync(highGoalToConeStack);
                            conesScored += 1;
                        }
                    }
                    break;
                case HG_TO_CS:
                    if (conesScored == CONE_COUNT) {
                        slides.rest();
                        arm.intakePos();
                        clamp.open();
                        trajectoryState = TrajectoryState.PARK;
                    } else {
                        runThread(pickupReadyThread);
                        if (!drive.isBusy()) {
                            runThread(pickupThread);
                            if (time.seconds() >= AutoConstants.DELAY_PICKUP) {
                                trajectoryState = TrajectoryState.CS_TO_HG;
                                drive.followTrajectorySequenceAsync(coneStackToHighGoal);
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
                    break;
            }
        }
    }
}
