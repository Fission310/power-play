package org.firstinspires.ftc.teamcode.opmode.auton.odometry.midgoal;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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

@Autonomous (name = "MID 5 Cone Auto", group = "_ared")
public class FiveConeAuto extends LinearOpMode {

    private Arm arm;
    private SlidesMotors slides;
    private final SignalSleeveWebcam signalSleeveWebcam = new SignalSleeveWebcam(this, "rightWebcam", SignalSleeveWebcam.ROBOT_SIDE.CONTROL_HUB);

    ElapsedTime time = new ElapsedTime();

    private boolean canContinue = false;
    private boolean canSlidesExtend = false;

    private static final double DELAY_PRELOAD_PICKUP = 4.5;
    public static final double DELAY_PICKUP = 2.5;

    public Runnable scoreReady = () -> {
        try {
            Thread.sleep(150);
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

    private static final TrajectoryVelocityConstraint VELO = SampleMecanumDrive.getVelocityConstraint(36, Math.toRadians(250), Math.toRadians(250));
    private static final TrajectoryAccelerationConstraint ACCEL = SampleMecanumDrive.getAccelerationConstraint(36);

    private static final TrajectoryVelocityConstraint FAST_VELO = SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(250), Math.toRadians(250));
    private static final TrajectoryAccelerationConstraint FAST_ACCEL = SampleMecanumDrive.getAccelerationConstraint(40);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Clamp clamp = new Clamp(this);
        arm = new Arm(this);
        slides = new SlidesMotors(this);
        signalSleeveWebcam.init(hardwareMap);
        clamp.init(hardwareMap);
        arm.init(hardwareMap);
        slides.init(hardwareMap);

        Thread scoreReadyThread = new Thread(scoreReady);
        Thread scoreThread = new Thread(score);

        drive.setPoseEstimate(AutoConstants.RR_START_POSE);

        conesScored = 0;

        TrajectorySequence preload = drive.trajectorySequenceBuilder(AutoConstants.RR_START_POSE)
                .setConstraints(VELO, ACCEL)
                .lineToLinearHeading(AutoConstants.RR_ODO_PRELOAD_MID_GOAL_POSE)
                .lineToConstantHeading(AutoConstants.RR_ODO_PRELOAD_MID_GOAL_VECTOR)
                .build();

        TrajectorySequence preloadToConeStack = drive.trajectorySequenceBuilder(preload.end())
                .setConstraints(VELO, ACCEL)
                .lineToLinearHeading(AutoConstants.RR_ODO_PRELOAD_MID_GOAL_POSE)
                .lineToLinearHeading(AutoConstants.RR_ODO_PRELOAD_CONE_STACK_POSE)
                .lineToConstantHeading(AutoConstants.RR_ODO_CONE_STACK_VECTOR)
                .build();

        TrajectorySequence coneStackToHighGoal = drive.trajectorySequenceBuilder(preloadToConeStack.end())
                .setConstraints(VELO, ACCEL)
                .setReversed(true)
                .setTangent(AutoConstants.RR_HEADING)
                .splineTo(AutoConstants.RR_ODO_MID_GOAL_VECTOR, AutoConstants.RR_ODO_MID_GOAL_HEADING)
                .build();

        TrajectorySequence highGoalToConeStack = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setConstraints(VELO, ACCEL)
                .setReversed(false)
                .setTangent(AutoConstants.RR_ODO_MID_CONE_STACK_TANGENT)
                .splineTo(AutoConstants.RR_ODO_CONE_STACK_VECTOR, AutoConstants.RR_ODO_CONE_STACK_HEADING)
                .build();

        TrajectorySequence toParkTemp = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setConstraints(VELO, ACCEL)
                .setReversed(false)
                .setTangent(AutoConstants.RR_ODO_CONE_STACK_TANGENT)
                .splineTo(AutoConstants.RR_ODO_MIDDLE_PARK_VECTOR, AutoConstants.RR_ODO_MID_MIDDLE_PARK_HEADING)
                .build();

        TrajectorySequence toLeftPark = drive.trajectorySequenceBuilder(toParkTemp.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .lineToLinearHeading(AutoConstants.RR_ODO_LEFT_PARK_POSE)
                .back(9)
                .build();

        TrajectorySequence toMiddlePark = drive.trajectorySequenceBuilder(toParkTemp.end())
                .setConstraints(VELO, ACCEL)
                .strafeRight(1)
                .build();

        TrajectorySequence toRightPark = drive.trajectorySequenceBuilder(toParkTemp.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .lineToLinearHeading(AutoConstants.RR_ODO_RIGHT_PARK_POSE)
                .back(9)
                .build();

        clamp.close();
        arm.intakePos();

        waitForStart();

        SignalSleeveWebcam.Side parkSide = signalSleeveWebcam.side();

        signalSleeveWebcam.stopStreaming();

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
                        if (time.seconds() >= DELAY_PRELOAD_PICKUP) {
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
                            if (conesScored >= CONE_COUNT) {
                                slides.rest();
                                arm.intakePos();
                                clamp.open();
                                time.reset();
                                drive.followTrajectorySequenceAsync(toParkTemp);
                                trajectoryState = TrajectoryState.PARK;
                            } else {
                                drive.followTrajectorySequenceAsync(highGoalToConeStack);
                                trajectoryState = TrajectoryState.HG_TO_CS;
                            }
                        }
                    }
                    break;
                case HG_TO_CS:
                    if (time.seconds() > AutoConstants.DELAY_OPEN_CLAMP) {
                        clamp.intakePos();
                    }
                    if (!drive.isBusy()) {
                        Pose2d currPose = drive.getPoseEstimate();
                        drive.setPoseEstimate(new Pose2d(currPose.getX(), currPose.getY() + 0.085, currPose.getHeading()));
                        clamp.close();
                        if (canSlidesExtend) {
                            slides.extendHighAuto();
                            canSlidesExtend = false;
                        }
                        if (time.seconds() >= DELAY_PICKUP) {
                            arm.autoScorePos();
                            canContinue = false;
                            canSlidesExtend = true;
                            drive.followTrajectorySequenceAsync(coneStackToHighGoal);
                            trajectoryState = TrajectoryState.CS_TO_HG;
                        }
                    }
                    break;
                case PARK:
                    switch (parkSide) {
                        case THREE:
                            if (!drive.isBusy()) {
                                clamp.close();
                                drive.followTrajectorySequenceAsync(toRightPark);
                                trajectoryState = TrajectoryState.IDLE;
                                time.reset();
                            }
                            break;
                        case TWO:
                            if (!drive.isBusy()) {
                                clamp.close();
                                drive.followTrajectorySequenceAsync(toMiddlePark);
                                trajectoryState = TrajectoryState.IDLE;
                                time.reset();
                            }
                            break;
                        case ONE:
                            if (!drive.isBusy()) {
                                clamp.close();
                                drive.followTrajectorySequenceAsync(toLeftPark);
                                trajectoryState = TrajectoryState.IDLE;
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
                    arm.scorePos();
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
