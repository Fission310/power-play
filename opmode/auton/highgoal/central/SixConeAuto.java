package org.firstinspires.ftc.teamcode.opmode.auton.highgoal.central;

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

@Autonomous (name = "CENTRAL HIGH 6 Cone Auto", group = "_yred", preselectTeleOp = "Drift Comp Main")
public class SixConeAuto extends LinearOpMode {

    private Arm arm;
    private SlidesMotors slides;
    private final SignalSleeveWebcam signalSleeveWebcam = new SignalSleeveWebcam(this, "rightWebcam", SignalSleeveWebcam.ROBOT_SIDE.CONTROL_HUB);

    ElapsedTime time = new ElapsedTime();

    private boolean canContinue = false;
    private boolean canSlidesExtend = false;

    private static final double DELAY_PRELOAD_PICKUP = 3.9;
    public static final double DELAY_PICKUP = 2.1;
    public static final double DELAY_SLIDES_FULL_EXTEND = 1.5;

    public Runnable scoreReady = () -> {
        try {
            Thread.sleep(2000);
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
            Thread.sleep(100);
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
    private static final int CONE_COUNT = 6;
    private static int conesScored;

//    private static final TrajectoryVelocityConstraint VELO = SampleMecanumDrive.getVelocityConstraint(43, Math.toRadians(250), Math.toRadians(250));
//    private static final TrajectoryAccelerationConstraint ACCEL = SampleMecanumDrive.getAccelerationConstraint(43);

    private static final TrajectoryVelocityConstraint FAST_VELO = SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(250), Math.toRadians(250));
    private static final TrajectoryAccelerationConstraint FAST_ACCEL = SampleMecanumDrive.getAccelerationConstraint(52);

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
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .lineToLinearHeading(AutoConstants.RR_ODO_PRELOAD_CONE_STACK_POSE)
                .waitSeconds(0.01)
                .splineTo(AutoConstants.C_HIGH_GOAL_VECTOR, Math.toRadians(215))
                .build();

        TrajectorySequence preloadToConeStack = drive.trajectorySequenceBuilder(preload.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .setReversed(true)
                .setTangent(AutoConstants.RR_ODO_MID_CONE_STACK_TANGENT)
                .splineTo(AutoConstants.RR_ODO_PRELOAD_CONE_STACK_VECTOR, Math.toRadians(0))
                .lineToConstantHeading(AutoConstants.RR_ODO_CONE_STACK_VECTOR)
                .build();

        TrajectorySequence coneStackToHighGoal = drive.trajectorySequenceBuilder(preloadToConeStack.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .setReversed(false)
                .lineToLinearHeading(AutoConstants.RR_ODO_PRELOAD_CONE_STACK_POSE)
                .splineTo(AutoConstants.C_HIGH_GOAL_VECTOR, Math.toRadians(215))
                .build();

        TrajectorySequence highGoalToConeStack = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .setReversed(true)
                .setTangent(AutoConstants.RR_ODO_MID_CONE_STACK_TANGENT)
                .splineTo(AutoConstants.RR_ODO_PRELOAD_CONE_STACK_VECTOR, Math.toRadians(0))
                .lineToConstantHeading(AutoConstants.RR_ODO_CONE_STACK_VECTOR)
                .build();

        TrajectorySequence toLeftPark = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .back(9)
                .turn(Math.toRadians(-35))
                .build();

        TrajectorySequence toMiddlePark = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .setReversed(true)
                .setTangent(AutoConstants.RR_ODO_MID_CONE_STACK_TANGENT)
                .splineTo(AutoConstants.RR_ODO_PRELOAD_CONE_STACK_VECTOR, Math.toRadians(0))
                .build();

        TrajectorySequence toRightPark = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .setReversed(true)
                .setTangent(AutoConstants.RR_ODO_MID_CONE_STACK_TANGENT)
                .splineTo(AutoConstants.RR_ODO_PRELOAD_CONE_STACK_VECTOR, Math.toRadians(0))
                .lineToConstantHeading(AutoConstants.RR_ODO_CONE_STACK_VECTOR)
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
                        Pose2d currPose = drive.getPoseEstimate();
                        drive.setPoseEstimate(new Pose2d(currPose.getX(), currPose.getY() + 0.15, currPose.getHeading()));
                        clamp.close();
                        if (canSlidesExtend) {
                            slides.extendPrepArm();
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
                    if (canSlidesExtend && time.seconds() >= DELAY_SLIDES_FULL_EXTEND) {
                        slides.extendHighAuto();
                        canSlidesExtend = false;
                    }
                    if (!drive.isBusy()) {
                        slides.extendToPosition(slides.getPosition() - 3);
                        canSlidesExtend = true;
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
                        drive.setPoseEstimate(new Pose2d(currPose.getX(), currPose.getY() + 0.205, currPose.getHeading()));
                        clamp.close();
                        if (canSlidesExtend) {
                            slides.extendPrepArm();
                            canSlidesExtend = false;
                        }
                        if (time.seconds() >= DELAY_PICKUP) {
                            arm.autoScorePos();
                            canContinue = false;
                            canSlidesExtend = true;
                            drive.followTrajectorySequenceAsync(coneStackToHighGoal);
                            time.reset();
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
                    arm.autoConeStackPos();
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
