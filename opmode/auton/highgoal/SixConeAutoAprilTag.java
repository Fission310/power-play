package org.firstinspires.ftc.teamcode.opmode.auton.highgoal;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Clamp;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.SignalSleeveWebcam;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.SlidesMotors;
import org.firstinspires.ftc.teamcode.opmode.auton.AutoConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "HIGH 6 Cone Auto", group = "_ared", preselectTeleOp = "Drift Comp Main")
public class SixConeAutoAprilTag extends LinearOpMode {

    private Arm arm;
    private Clamp clamp;
    private SlidesMotors slides;
    private final SignalSleeveWebcam signalSleeveWebcam = new SignalSleeveWebcam(this, "rightWebcam", SignalSleeveWebcam.ROBOT_SIDE.CONTROL_HUB);

    ElapsedTime time = new ElapsedTime();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    final int Left = 1;
    final int Middle = 2;
    final int Right = 3;

    AprilTagDetection tagOfInterest = null;

    private boolean canContinue = false;
    private boolean canSlidesExtend = false;

    private static final double DELAY_PRELOAD_PICKUP = 3.9;
    public static final double DELAY_PICKUP = 2.1;

    public Runnable scoreReady = () -> {
        try {
            clamp.close();
            Thread.sleep(500);
            clamp.close();
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

    private static final TrajectoryVelocityConstraint VELO = SampleMecanumDrive.getVelocityConstraint(43, Math.toRadians(250), Math.toRadians(250));
    private static final TrajectoryAccelerationConstraint ACCEL = SampleMecanumDrive.getAccelerationConstraint(43);

    private static final TrajectoryVelocityConstraint FAST_VELO = SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(250), Math.toRadians(250));
    private static final TrajectoryAccelerationConstraint FAST_ACCEL = SampleMecanumDrive.getAccelerationConstraint(52);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        clamp = new Clamp(this);
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
                .lineToLinearHeading(AutoConstants.RR_ODO_PRELOAD_HIGH_GOAL_POSE_SIX)
                .lineToConstantHeading(AutoConstants.RR_ODO_PRELOAD_HIGH_GOAL_VECTOR_SIX)
                .build();

        TrajectorySequence preloadToConeStack = drive.trajectorySequenceBuilder(preload.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .lineToLinearHeading(AutoConstants.RR_ODO_PRELOAD_HIGH_GOAL_POSE_SIX)
                .lineToLinearHeading(AutoConstants.RR_ODO_PRELOAD_CONE_STACK_POSE_SIX)
                .setConstraints(VELO, ACCEL)
                .lineToConstantHeading(AutoConstants.RR_ODO_CONE_STACK_VECTOR_SIX)
                .build();

        TrajectorySequence coneStackToHighGoal = drive.trajectorySequenceBuilder(preloadToConeStack.end())
                .setConstraints(VELO, ACCEL)
                .setReversed(true)
                .setTangent(AutoConstants.RR_HEADING)
                .splineTo(AutoConstants.RR_ODO_HIGH_GOAL_VECTOR_SIX, AutoConstants.RR_ODO_HIGH_GOAL_HEADING)
                .build();

        TrajectorySequence highGoalToConeStack = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setConstraints(VELO, ACCEL)
                .setReversed(false)
                .setTangent(AutoConstants.RR_ODO_CONE_STACK_TANGENT)
                .splineTo(AutoConstants.RR_ODO_CONE_STACK_VECTOR_SIX, AutoConstants.RR_ODO_CONE_STACK_HEADING)
                .build();

        TrajectorySequence toParkTemp = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setConstraints(VELO, ACCEL)
                .setReversed(false)
                .setTangent(AutoConstants.RR_ODO_CONE_STACK_TANGENT)
                .splineTo(AutoConstants.RR_ODO_MIDDLE_PARK_VECTOR, AutoConstants.RR_ODO_MIDDLE_PARK_HEADING)
                .build();

        TrajectorySequence toLeftPark = drive.trajectorySequenceBuilder(toParkTemp.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .lineToLinearHeading(AutoConstants.RR_ODO_LEFT_PARK_POSE)
                .back(9)
                .build();

        TrajectorySequence toMiddlePark = drive.trajectorySequenceBuilder(toParkTemp.end())
                .setConstraints(VELO, ACCEL)
//                .strafeLeft(2)
                .lineToLinearHeading(new Pose2d(AutoConstants.RR_ODO_MIDDLE_PARK_VECTOR.getX() - 2, AutoConstants.RR_ODO_MIDDLE_PARK_VECTOR.getY(), Math.toRadians(90)))
                .build();

        TrajectorySequence toRightPark = drive.trajectorySequenceBuilder(toParkTemp.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .lineToLinearHeading(AutoConstants.RR_ODO_RIGHT_PARK_POSE)
                .back(9)
                .build();

        clamp.close();
        arm.intakePos();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == Left || tag.id == Middle || tag.id == Right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        camera.stopStreaming();

        drive.followTrajectorySequenceAsync(preload);
        runThread(scoreReadyThread);
        clamp.close();

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
                        drive.setPoseEstimate(new Pose2d(currPose.getX(), currPose.getY() + 0.197, currPose.getHeading()));
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
                    if (tagOfInterest == null || tagOfInterest.id == Right) {
                        if (!drive.isBusy()) {
                            clamp.close();
                            drive.followTrajectorySequenceAsync(toRightPark);
                            trajectoryState = TrajectoryState.IDLE;
                            time.reset();
                        }
                    } else if (tagOfInterest.id == Middle) {
                        if (!drive.isBusy()) {
                            clamp.close();
                            drive.followTrajectorySequenceAsync(toMiddlePark);
                            trajectoryState = TrajectoryState.IDLE;
                            time.reset();
                        }
                    } else if (tagOfInterest.id == Left) {
                        if (!drive.isBusy()) {
                            clamp.close();
                            drive.followTrajectorySequenceAsync(toLeftPark);
                            trajectoryState = TrajectoryState.IDLE;
                            time.reset();
                        }
                    }
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

    @SuppressLint ("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
