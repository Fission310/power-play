package org.firstinspires.ftc.teamcode.opmode.encoder.highgoal;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Clamp;
import org.firstinspires.ftc.teamcode.hardware.SignalSleeveWebcam;
import org.firstinspires.ftc.teamcode.hardware.SlidesMotors;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.opmode.auton.AutoConstants;

@Disabled
@Autonomous (name = "ENCODER 5 Cone Auto", group = "_bred")
public class FiveConeAuto extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Clamp clamp;
    private Arm arm;
    private SlidesMotors slides;
    private SignalSleeveWebcam signalSleeveWebcam = new SignalSleeveWebcam(this, "rightWebcam", SignalSleeveWebcam.ROBOT_SIDE.CONTROL_HUB);

    private SignalSleeveWebcam.Side parkSide = SignalSleeveWebcam.Side.ONE;

    private Thread scoreReadyThread;
    private Thread scoreThread;

    ElapsedTime time = new ElapsedTime();

    private TrajectorySequence preloadToConeStack;
    private TrajectorySequence coneStackToHighGoal;
    private TrajectorySequence highGoalToConeStack;

    private boolean canContinue = false;
    private boolean canSlidesExtend = false;

    private static double RR_CONE_STACK_X = 60.5;
    private static double RR_CONE_STACK_Y = -17.2;
    public static double RR_PRELOAD_CONE_STACK_Y = -17;
    public static Vector2d RR_PRELOAD_CONE_STACK_VECTOR = new Vector2d(RR_CONE_STACK_X, RR_PRELOAD_CONE_STACK_Y);

    public static Vector2d RR_CONE_STACK_VECTOR = new Vector2d(RR_CONE_STACK_X, RR_CONE_STACK_Y);

    public Runnable scoreReady = () -> {
        try {
            // extend slides lvl 3
            // rotate arm to intake pos
            slides.extendHighAuto();
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
        SCORING,
        HG_TO_CS,
        PARK,
        IDLE
    }
    TrajectoryState trajectoryState = TrajectoryState.PRELOAD;

    /** VERY IMPORTANT **/
    private static final int CONE_COUNT = 5;
    private static int conesScored;

    public static final Pose2d RR_PARK_LEFT = new Pose2d(AutoConstants.RR_PARK_LEFT_X, AutoConstants.RR_CONE_STACK_Y, AutoConstants.RR_HEADING);
    public static final Pose2d RR_PARK_MIDDLE = new Pose2d(AutoConstants.RR_CENTER_X, AutoConstants.RR_CONE_STACK_Y, AutoConstants.RR_HEADING);

    // 36 | 36 WORKS WORKS WORKS WORKS HAHA!

    private static final TrajectoryVelocityConstraint VELO = SampleMecanumDrive.getVelocityConstraint(36, Math.toRadians(250), Math.toRadians(250));
    private static final TrajectoryAccelerationConstraint ACCEL = SampleMecanumDrive.getAccelerationConstraint(36);

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

        drive.setPoseEstimate(AutoConstants.RR_START_POSE);

        conesScored = 0;

        TrajectorySequence preload = drive.trajectorySequenceBuilder(AutoConstants.RR_START_POSE)
                .setConstraints(VELO, ACCEL)
                .setTangent(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(AutoConstants.RR_CENTER_X, AutoConstants.RR_HIGH_GOAL_Y - AutoConstants.RR_HIGH_GOAL_Y_PRELOAD_OFFSET, AutoConstants.RR_HEADING))
                .lineToLinearHeading(new Pose2d(AutoConstants.RR_HIGH_GOAL_X + AutoConstants.RR_PRELOAD_X_OFFSET, AutoConstants.RR_HIGH_GOAL_Y - AutoConstants.RR_HIGH_GOAL_Y_PRELOAD_OFFSET, AutoConstants.RR_HEADING))
//                .waitSeconds(0.1) // 0.1
                .build();

        preloadToConeStack = drive.trajectorySequenceBuilder(preload.end())
                .setConstraints(VELO, ACCEL)
                .lineToLinearHeading(new Pose2d(AutoConstants.RR_CENTER_X, AutoConstants.RR_HIGH_GOAL_Y, AutoConstants.RR_HEADING))

                .lineToLinearHeading(new Pose2d(AutoConstants.RR_CENTER_X, AutoConstants.RR_PRELOAD_CONE_STACK_Y + 1, AutoConstants.RR_HEADING))
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.RR_CONE_STACK_ANGLE + AutoConstants.RR_CONE_STACK_ANGLE_OFFSET))

                .splineToConstantHeading(RR_PRELOAD_CONE_STACK_VECTOR, Math.toRadians(AutoConstants.RR_CONE_STACK_END_ANGLE))
                .build();

        coneStackToHighGoal = drive.trajectorySequenceBuilder(preloadToConeStack.end())
                .setConstraints(VELO, ACCEL)
                .setReversed(false)
                .setTangent(Math.toRadians(AutoConstants.RR_HIGH_GOAL_TANGENT))
                .splineTo(AutoConstants.RR_HIGH_GOAL_VECTOR, Math.toRadians(AutoConstants.RR_HIGH_GOAL_ANGLE))
                .build();

        highGoalToConeStack = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setConstraints(VELO, ACCEL)
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.RR_CONE_STACK_ANGLE))
                .splineTo(RR_CONE_STACK_VECTOR, Math.toRadians(AutoConstants.RR_CONE_STACK_END_ANGLE))
                .build();

        TrajectorySequence toLeftPark = drive.trajectorySequenceBuilder(highGoalToConeStack.end())
                .setConstraints(VELO, ACCEL)
                .lineToLinearHeading(RR_PARK_LEFT)
                .build();

        TrajectorySequence toMiddlePark = drive.trajectorySequenceBuilder(highGoalToConeStack.end())
                .setConstraints(VELO, ACCEL)
                .lineToLinearHeading(RR_PARK_MIDDLE)
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
            telemetry.update();
            drive.update();
            slides.update();

            switch (trajectoryState) {
                case PRELOAD:
                    if (!drive.isBusy()) {
                        slides.extendToPosition(slides.getPosition() - 3);
                        clamp.open();
                        runThread(scoreThread);
                        if (canContinue) {
                            conesScored += 1;
                            slides.extendToPosition(AutoConstants.SLIDE_EXTEND_POSITIONS[conesScored]);
                            canContinue = false;
                            canSlidesExtend = true;
                            time.reset();
                            drive.followTrajectorySequenceAsync(preloadToConeStack);
                            trajectoryState = TrajectoryState.PRELOAD_TO_CS;
                        }
                    }
                    break;
                case PRELOAD_TO_CS:
                    if (time.seconds() > AutoConstants.DELAY_OPEN_CLAMP) {
                        clamp.intakePos();
                    }
                    if (!drive.isBusy()) {
                        clamp.close();
                        if (canSlidesExtend) {
                            slides.extendHighAuto();
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
                        slides.extendToPosition(slides.getPosition() - 3);
                        trajectoryState = TrajectoryState.SCORING;
                        time.reset();
                    }
                    break;
                case SCORING:
                    if (time.seconds() >= AutoConstants.DELAY_SCORING) {
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
                        time.reset();
                        trajectoryState = TrajectoryState.PARK;
                    } else {
                        if (time.seconds() > AutoConstants.DELAY_OPEN_CLAMP) {
                            clamp.intakePos();
                        }
                        if (!drive.isBusy()) {
//
//                            if (coneSensor.getDistanceMM() >= 80) {
                                Pose2d currPose = drive.getPoseEstimate();
                                drive.setPoseEstimate(new Pose2d(currPose.getX(), currPose.getY() - 0.1, currPose.getHeading()));
//                            }

                            clamp.close();
                            if (canSlidesExtend) {
                                slides.extendHighAuto();
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
                    if (time.seconds() > AutoConstants.DELAY_OPEN_CLAMP) {
                        clamp.intakePos();
                    }
                    switch (parkSide) {
                        case THREE:
                            if (!drive.isBusy()) {
                                clamp.close();
                                trajectoryState = TrajectoryState.IDLE;
                                time.reset();
                            }
                            break;
                        case TWO:
                            if (!drive.isBusy()) {
                                clamp.close();
                                trajectoryState = TrajectoryState.IDLE;
                                drive.followTrajectorySequenceAsync(toMiddlePark);
                                time.reset();
                            }
                            break;
                        case ONE:
                            if (!drive.isBusy()) {
                                clamp.close();
                                trajectoryState = TrajectoryState.IDLE;
                                drive.followTrajectorySequenceAsync(toLeftPark);
                                time.reset();
                            }
                            break;
                        case NOT_FOUND:
                        default:
                            if (!drive.isBusy()) {
                                trajectoryState = TrajectoryState.IDLE;
                                time.reset();
                            }
                            break;
                    }
                    break;
                case IDLE:
                    arm.groundScorePos();
                    clamp.close();
                    if (time.seconds() > 1.5) {
                        if (!drive.isBusy()) {
                            stop();
                        }
                    }
                    break;
            }
        }
    }
}
