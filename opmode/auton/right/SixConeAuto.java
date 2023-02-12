package org.firstinspires.ftc.teamcode.opmode.auton.right;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous (name = "RIGHT_SIDE 6 Cone Auto", group = "_ared")
public class SixConeAuto extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Clamp clamp;
    private Arm arm;
    private SlidesMotors slides;
    private SignalSleeveWebcam signalSleeveWebcam = new SignalSleeveWebcam(this, "rightWebcam", SignalSleeveWebcam.ROBOT_SIDE.CONTROL_HUB);

    private SignalSleeveWebcam.Side parkSide = SignalSleeveWebcam.Side.ONE;

    private Thread scoreReadyThread;
    private Thread scoreThread;

    ElapsedTime time = new ElapsedTime();
//    ElapsedTime looptimer = new ElapsedTime();

    private TrajectorySequence preloadToConeStack;
    private TrajectorySequence coneStackToHighGoal;
    private TrajectorySequence highGoalToConeStack;

    private boolean canContinue = false;
    private boolean canSlidesExtend = false;

    private static final double DELAY_PRELOAD_PICKUP = 4;
    public static final double DELAY_PICKUP = 2.172;

    public Runnable scoreReady = () -> {
        try {
            Thread.sleep(450);
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
    private static final int CONE_COUNT = 6;
    private static int conesScored;


//    private static final double DRIFT_AMT_Y = 2; //
//    private static final double DRIFT_AMT_X = 0.05;

    public static final Pose2d RR_PARK_LEFT = new Pose2d(AutoConstants.RR_PARK_LEFT_X, AutoConstants.RR_CONE_STACK_Y - 1.4, Math.toRadians(90));
    public static final Pose2d RR_PARK_MIDDLE = new Pose2d(AutoConstants.RR_CENTER_X - 1, AutoConstants.RR_CONE_STACK_Y, Math.toRadians(90));

    // 36 | 36 WORKS WORKS WORKS WORKS HAHA!
    private static final TrajectoryVelocityConstraint VELO = SampleMecanumDrive.getVelocityConstraint(38, Math.toRadians(250), Math.toRadians(250));
    private static final TrajectoryAccelerationConstraint ACCEL = SampleMecanumDrive.getAccelerationConstraint(38);

    private static final TrajectoryVelocityConstraint FAST_VELO = SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(250), Math.toRadians(250));
    private static final TrajectoryAccelerationConstraint FAST_ACCEL = SampleMecanumDrive.getAccelerationConstraint(60);

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
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .setTangent(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(AutoConstants.RR_CENTER_X, AutoConstants.RR_HIGH_GOAL_Y - AutoConstants.RR_HIGH_GOAL_Y_PRELOAD_OFFSET, AutoConstants.RR_HEADING))
                .lineToLinearHeading(new Pose2d(AutoConstants.RR_HIGH_GOAL_X + AutoConstants.RR_PRELOAD_X_OFFSET, AutoConstants.RR_HIGH_GOAL_Y - AutoConstants.RR_HIGH_GOAL_Y_PRELOAD_OFFSET, AutoConstants.RR_HEADING))
                .build();

        preloadToConeStack = drive.trajectorySequenceBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(AutoConstants.RR_CENTER_X, AutoConstants.RR_HIGH_GOAL_Y, AutoConstants.RR_HEADING))
//
                .lineToLinearHeading(new Pose2d(AutoConstants.RR_CENTER_X, AutoConstants.RR_PRELOAD_CONE_STACK_Y + 1, AutoConstants.RR_HEADING))
                .setReversed(true)
//                .setTangent(Math.toRadians(265))
                .setTangent(Math.toRadians(AutoConstants.RR_CONE_STACK_ANGLE + AutoConstants.RR_CONE_STACK_ANGLE_OFFSET))
                .setConstraints(VELO, ACCEL)
                .splineToConstantHeading(AutoConstants.RR_PRELOAD_CONE_STACK_VECTOR, Math.toRadians(AutoConstants.RR_CONE_STACK_END_ANGLE))
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
                .splineTo(AutoConstants.RR_CONE_STACK_VECTOR, Math.toRadians(AutoConstants.RR_CONE_STACK_END_ANGLE))
                .build();

        TrajectorySequence toParkTemp = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setConstraints(VELO, ACCEL)

//                .setTangent(Math.toRadians(315))
//                .splineTo(new Vector2d(AutoConstants.RR_CENTER_X + 7, AutoConstants.RR_CONE_STACK_Y + 2), Math.toRadians(0))

                .setTangent(Math.toRadians(315))
                .splineTo(new Vector2d(AutoConstants.RR_CENTER_X - 4, AutoConstants.RR_CONE_STACK_Y), Math.toRadians(270))

                .build();

        TrajectorySequence toLeftPark = drive.trajectorySequenceBuilder(toParkTemp.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .lineToLinearHeading(RR_PARK_LEFT)
                .build();

        TrajectorySequence toMiddlePark = drive.trajectorySequenceBuilder(toParkTemp.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
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
//        looptimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
//            double starttime = looptimer.time();
//            telemetry.addData("cones scored", conesScored);
//            telemetry.addData("curr state", trajectoryState);
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
//                            if (parkSide == SignalSleeveWebcam.Side.ONE) {
//                                drive.followTrajectorySequenceAsync(toParkTemp);
//                                trajectoryState = TrajectoryState.PARK;
//                            }
                            if ((conesScored >= CONE_COUNT) && ((parkSide == SignalSleeveWebcam.Side.ONE) || (parkSide == SignalSleeveWebcam.Side.TWO))) {
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
                    arm.intakePos();
                    clamp.close();
                    if (time.seconds() > 1.5) {
                        if (!drive.isBusy()) {
                            stop();
                        }
                    }
                    break;
            }
//            double endtime = looptimer.time();
//            telemetry.addData("loop time", endtime-starttime);
//            looptimer.reset();
        }
    }
}
