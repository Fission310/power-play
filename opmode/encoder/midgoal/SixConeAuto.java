package org.firstinspires.ftc.teamcode.opmode.encoder.midgoal;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Clamp;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.SignalSleeveWebcam;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.SlidesMotors;
import org.firstinspires.ftc.teamcode.opmode.auton.AutoConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous (name = "ENCODER MID 5 CONE AUTO", group = "_ared")
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

    private boolean canContinue = false;
    private boolean canSlidesExtend = false;

    private static double RR_CONE_STACK_X = 59.75;
    private static double RR_CONE_STACK_Y = -16.6;
    public static double RR_PRELOAD_CONE_STACK_Y = -16;
    public static Vector2d RR_PRELOAD_CONE_STACK_VECTOR = new Vector2d(RR_CONE_STACK_X, RR_PRELOAD_CONE_STACK_Y);

    public static Vector2d RR_CONE_STACK_VECTOR = new Vector2d(RR_CONE_STACK_X, RR_CONE_STACK_Y);

    public Runnable scoreReady = () -> {
        try {
            // extend slides lvl 3
            // rotate arm to intake pos
            slides.extendMedium();
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
        CS_TO_MG,
        SCORING,
        MG_TO_CS,
        PARK,
        IDLE
    }
    TrajectoryState trajectoryState = TrajectoryState.PRELOAD;

    /** VERY IMPORTANT **/
    private static final int CONE_COUNT = 5;
    private static int conesScored;

    public static final Pose2d RR_PARK_LEFT = new Pose2d(AutoConstants.RR_PARK_LEFT_X, RR_CONE_STACK_Y, AutoConstants.RR_HEADING);
    public static final Pose2d RR_PARK_MIDDLE = new Pose2d(AutoConstants.RR_CENTER_X, RR_CONE_STACK_Y, AutoConstants.RR_HEADING);

    private static final TrajectoryVelocityConstraint FAST_VELO = SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(250), Math.toRadians(250));
    private static final TrajectoryAccelerationConstraint FAST_ACCEL = SampleMecanumDrive.getAccelerationConstraint(50);

    private static final TrajectoryVelocityConstraint VELO = SampleMecanumDrive.getVelocityConstraint(37, Math.toRadians(250), Math.toRadians(250));
    private static final TrajectoryAccelerationConstraint ACCEL = SampleMecanumDrive.getAccelerationConstraint(37);

    public static final double DELAY_PRELOAD_PICKUP = 3.92; // 4.15
    public static final double DELAY_PICKUP = 2.53; // 2.52

    double stackSplineOffset = 6;

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

        ////

        TrajectorySequence preload = drive.trajectorySequenceBuilder(AutoConstants.RR_START_POSE)
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .setTangent(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(AutoConstants.RR_CENTER_X, AutoConstants.RR_MED_GOAL_Y - 2.4, Math.toRadians(180)))
                .lineTo(new Vector2d(AutoConstants.RR_MED_GOAL_X + 0.5, AutoConstants.RR_MED_GOAL_Y - 2.4))

                .build();

        TrajectorySequence preloadToConeStack = drive.trajectorySequenceBuilder(preload.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .lineToLinearHeading(new Pose2d(AutoConstants.RR_CENTER_X, AutoConstants.RR_MED_GOAL_Y - 2.4, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(AutoConstants.RR_CENTER_X, RR_PRELOAD_CONE_STACK_Y, Math.toRadians(180)))

                .setConstraints(VELO, ACCEL)
                .setTangent(Math.toRadians(AutoConstants.RR_CONE_STACK_ANGLE + AutoConstants.RR_CONE_STACK_ANGLE_OFFSET + 25))
                .splineToConstantHeading(RR_CONE_STACK_VECTOR, Math.toRadians(AutoConstants.RR_CONE_STACK_END_ANGLE))

                .build();

        TrajectorySequence coneStackToMedGoal = drive.trajectorySequenceBuilder(preloadToConeStack.end())
                .setConstraints(VELO, ACCEL)
                .setReversed(false)
                .setTangent(Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(RR_CONE_STACK_X - stackSplineOffset, RR_CONE_STACK_Y, Math.toRadians(180)))
                .splineTo(AutoConstants.RR_MED_GOAL_VECTOR, Math.toRadians(225))
                
                .build();
        
        TrajectorySequence medGoalToConeStack = drive.trajectorySequenceBuilder(coneStackToMedGoal.end())
                .setConstraints(VELO, ACCEL)
                .setReversed(true)
                .setTangent(Math.toRadians(45))
                .splineTo(new Vector2d(RR_CONE_STACK_X - stackSplineOffset, RR_CONE_STACK_Y), Math.toRadians(0))
                .lineTo(RR_CONE_STACK_VECTOR)
                
                .build();

        TrajectorySequence toLeftPark = drive.trajectorySequenceBuilder(medGoalToConeStack.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .lineToLinearHeading(RR_PARK_LEFT)
                .build();

        TrajectorySequence toMiddlePark = drive.trajectorySequenceBuilder(medGoalToConeStack.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .lineToLinearHeading(RR_PARK_MIDDLE)
                .build();

        clamp.close();
        arm.intakePos();

        waitForStart();

        parkSide = signalSleeveWebcam.side();

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
                            slides.extendMedium();
                            canSlidesExtend = false;
                        }
                        if (time.seconds() >= DELAY_PRELOAD_PICKUP) {
                            arm.autoScorePos();
                            canContinue = false;
                            canSlidesExtend = true;
                            drive.followTrajectorySequenceAsync(coneStackToMedGoal);
                            trajectoryState = TrajectoryState.CS_TO_MG;
                        }
                    }
                    break;
                case CS_TO_MG:
                    if (!drive.isBusy()) {
                        Pose2d currPose = drive.getPoseEstimate();
                        drive.setPoseEstimate(new Pose2d(currPose.getX() + 0.07, currPose.getY(), currPose.getHeading()));
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
                            drive.followTrajectorySequenceAsync(medGoalToConeStack);
                            trajectoryState = TrajectoryState.MG_TO_CS;
                        }
                    }
                    break;
                case MG_TO_CS:
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
                            if (conesScored < 4) {
                                Pose2d currPose = drive.getPoseEstimate();
                                drive.setPoseEstimate(new Pose2d(currPose.getX() - 0.1, currPose.getY() - 0.1, currPose.getHeading() - Math.toRadians(0.1)));
                            }

                            clamp.close();
                            if (canSlidesExtend) {
                                slides.extendMedium();
                                canSlidesExtend = false;
                            }
                            if (time.seconds() >= DELAY_PICKUP) {
                                arm.autoScorePos();
                                canContinue = false;
                                canSlidesExtend = true;
                                drive.followTrajectorySequenceAsync(coneStackToMedGoal);
                                trajectoryState = TrajectoryState.CS_TO_MG;
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
