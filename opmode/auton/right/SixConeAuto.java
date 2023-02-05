package org.firstinspires.ftc.teamcode.opmode.auton.right;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous (name = "RIGHT_SIDE 6 Cone Auto", group = "red")
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

    private TrajectorySequence preloadToConeStack;
    private TrajectorySequence coneStackToHighGoal;
    private TrajectorySequence coneStackToHighGoalOne;
    private TrajectorySequence coneStackToHighGoalTwo;
    private TrajectorySequence coneStackToHighGoalThree;
    private TrajectorySequence coneStackToHighGoalFour;
    private TrajectorySequence highGoalToConeStack;
    private TrajectorySequence highGoalToConeStackOne;
    private TrajectorySequence highGoalToConeStackTwo;
    private TrajectorySequence highGoalToConeStackThree;
    private TrajectorySequence highGoalToConeStackFour;

    private boolean canContinue = false;
    private boolean canSlidesExtend = false;

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
    private static final int CONE_COUNT = 6;
    private static int conesScored;


    private static final double DRIFT_AMT_Y = 1.45; // 1.45 on 13V  ||  1.4 on 13.5V  ||  1.275 on 12.2V ??
    private static final double DRIFT_AMT_X = 0.1;

    public static final Pose2d RR_PARK_LEFT = new Pose2d(AutoConstants.RR_PARK_LEFT_X, AutoConstants.RR_CONE_STACK_Y + (DRIFT_AMT_Y * CONE_COUNT-2), AutoConstants.RR_HEADING);
    public static final Pose2d RR_PARK_MIDDLE = new Pose2d(AutoConstants.RR_CENTER_X, AutoConstants.RR_CONE_STACK_Y + (DRIFT_AMT_Y * CONE_COUNT-2), AutoConstants.RR_HEADING);


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        clamp = new Clamp(this);
        arm = new Arm(this);
        slides = new SlidesMotors(this);
//        signalSleeveWebcam.init(hardwareMap);

        clamp.init(hardwareMap);
        arm.init(hardwareMap);
        slides.init(hardwareMap);

        scoreReadyThread = new Thread(scoreReady);
        scoreThread = new Thread(score);

        drive.setPoseEstimate(AutoConstants.RR_START_POSE);

        conesScored = 0;

        TrajectorySequence preload = drive.trajectorySequenceBuilder(AutoConstants.RR_START_POSE)
                .setTangent(Math.toRadians(90))


                /**  TESTING  **/
//                .lineToLinearHeading(new Pose2d(AutoConstants.RR_CENTER_X, AutoConstants.RR_HIGH_GOAL_Y - AutoConstants.RR_HIGH_GOAL_Y_PRELOAD_OFFSET - 6, AutoConstants.RR_HEADING))
//                .splineToConstantHeading(new Vector2d(AutoConstants.RR_HIGH_GOAL_X + AutoConstants.RR_PRELOAD_X_OFFSET, AutoConstants.RR_HIGH_GOAL_Y), AutoConstants.RR_HEADING)
                /**  END TESTING  **/


                .lineToLinearHeading(new Pose2d(AutoConstants.RR_CENTER_X, AutoConstants.RR_HIGH_GOAL_Y - AutoConstants.RR_HIGH_GOAL_Y_PRELOAD_OFFSET, AutoConstants.RR_HEADING))
                .lineToLinearHeading(new Pose2d(AutoConstants.RR_HIGH_GOAL_X + AutoConstants.RR_PRELOAD_X_OFFSET, AutoConstants.RR_HIGH_GOAL_Y - AutoConstants.RR_HIGH_GOAL_Y_PRELOAD_OFFSET, AutoConstants.RR_HEADING))
                .waitSeconds(0.1) // 0.2
                .build();

        preloadToConeStack = drive.trajectorySequenceBuilder(preload.end())
                .lineToLinearHeading(new Pose2d(AutoConstants.RR_CENTER_X, AutoConstants.RR_HIGH_GOAL_Y, AutoConstants.RR_HEADING))

                /**  TESTING  **/
//                .setReversed(true)
//                .setTangent(Math.toRadians(270))
                /**  END TESTING  **/

                /** WORKING **/
                .lineToLinearHeading(new Pose2d(AutoConstants.RR_CENTER_X, AutoConstants.RR_PRELOAD_CONE_STACK_Y + 1, AutoConstants.RR_HEADING))
                .setReversed(true)
                /** END WORKING **/

                .setTangent(Math.toRadians(AutoConstants.RR_CONE_STACK_ANGLE + AutoConstants.RR_CONE_STACK_ANGLE_OFFSET))

                .splineToConstantHeading(AutoConstants.RR_PRELOAD_CONE_STACK_VECTOR, Math.toRadians(AutoConstants.RR_CONE_STACK_END_ANGLE))
                .build();

        coneStackToHighGoal = drive.trajectorySequenceBuilder(preloadToConeStack.end())
                .setReversed(false)
                .setTangent(Math.toRadians(AutoConstants.RR_HIGH_GOAL_TANGENT))
                .splineTo(AutoConstants.RR_HIGH_GOAL_VECTOR, Math.toRadians(AutoConstants.RR_HIGH_GOAL_ANGLE))
//                .waitSeconds(0.15) // 0.2 (try 0.15)
                .build();

        highGoalToConeStack = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.RR_CONE_STACK_ANGLE))
                .splineTo(AutoConstants.RR_CONE_STACK_VECTOR, Math.toRadians(AutoConstants.RR_CONE_STACK_END_ANGLE))
                .build();

        coneStackToHighGoalOne = drive.trajectorySequenceBuilder(highGoalToConeStack.end())
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.RR_HIGH_GOAL_TANGENT))
                .splineTo(new Vector2d(AutoConstants.RR_HIGH_GOAL_X - (DRIFT_AMT_X), AutoConstants.RR_HIGH_GOAL_Y + DRIFT_AMT_Y), Math.toRadians(AutoConstants.RR_HIGH_GOAL_ANGLE))
                // +0x, +0.55y
//                .waitSeconds(0.15)
                .build();

        highGoalToConeStackOne = drive.trajectorySequenceBuilder(coneStackToHighGoalOne.end())
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.RR_CONE_STACK_ANGLE))
                .splineTo(new Vector2d(AutoConstants.RR_CONE_STACK_X - 0.5, AutoConstants.RR_CONE_STACK_Y + DRIFT_AMT_Y), Math.toRadians(AutoConstants.RR_CONE_STACK_END_ANGLE))
                .build();

        coneStackToHighGoalTwo = drive.trajectorySequenceBuilder(highGoalToConeStackOne.end())
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.RR_HIGH_GOAL_TANGENT))
                .splineTo(new Vector2d(AutoConstants.RR_HIGH_GOAL_X - (DRIFT_AMT_X * 2), AutoConstants.RR_HIGH_GOAL_Y + (DRIFT_AMT_Y * 2)), Math.toRadians(AutoConstants.RR_HIGH_GOAL_ANGLE))
                // +0x, +0.55y
//                .waitSeconds(0.15)
                .build();

        highGoalToConeStackTwo = drive.trajectorySequenceBuilder(coneStackToHighGoalTwo.end())
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.RR_CONE_STACK_ANGLE))
                .splineTo(new Vector2d(AutoConstants.RR_CONE_STACK_X - 0.6, AutoConstants.RR_CONE_STACK_Y + (DRIFT_AMT_Y * 2)), Math.toRadians(AutoConstants.RR_CONE_STACK_END_ANGLE))
                .build();

        coneStackToHighGoalThree = drive.trajectorySequenceBuilder(highGoalToConeStackTwo.end())
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.RR_HIGH_GOAL_TANGENT))
                .splineTo(new Vector2d(AutoConstants.RR_HIGH_GOAL_X - (DRIFT_AMT_X * 3), AutoConstants.RR_HIGH_GOAL_Y + (DRIFT_AMT_Y * 3)), Math.toRadians(AutoConstants.RR_HIGH_GOAL_ANGLE))
                // +0x, +0.55y
//                .waitSeconds(0.15)
                .build();

        highGoalToConeStackThree = drive.trajectorySequenceBuilder(coneStackToHighGoalThree.end())
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.RR_CONE_STACK_ANGLE))
                .splineTo(new Vector2d(AutoConstants.RR_CONE_STACK_X - 1, AutoConstants.RR_CONE_STACK_Y + (DRIFT_AMT_Y * 3)), Math.toRadians(AutoConstants.RR_CONE_STACK_END_ANGLE))
                .build();

        coneStackToHighGoalFour = drive.trajectorySequenceBuilder(highGoalToConeStackThree.end())
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.RR_HIGH_GOAL_TANGENT))
                .splineTo(new Vector2d(AutoConstants.RR_HIGH_GOAL_X - (DRIFT_AMT_X * 4), AutoConstants.RR_HIGH_GOAL_Y + (DRIFT_AMT_Y * 4)), Math.toRadians(AutoConstants.RR_HIGH_GOAL_ANGLE))
                // +0x, +0.55y
//                .waitSeconds(0.15)
                .build();

        highGoalToConeStackFour = drive.trajectorySequenceBuilder(coneStackToHighGoalFour.end())
                .setReversed(true)
                .setTangent(Math.toRadians(AutoConstants.RR_CONE_STACK_ANGLE))
                .splineTo(new Vector2d(AutoConstants.RR_CONE_STACK_X - 1.2, AutoConstants.RR_CONE_STACK_Y + (DRIFT_AMT_Y * 4)), Math.toRadians(AutoConstants.RR_CONE_STACK_END_ANGLE))
                .build();

        //////////////////////////////



        TrajectorySequence toLeftPark = drive.trajectorySequenceBuilder(highGoalToConeStackFour.end())
                .lineToLinearHeading(RR_PARK_LEFT)
                .build();

        TrajectorySequence toMiddlePark = drive.trajectorySequenceBuilder(highGoalToConeStackFour.end())
                .lineToLinearHeading(RR_PARK_MIDDLE)
                .build();

        clamp.close();
        arm.autoIntakePos();

        waitForStart();

//        parkSide = signalSleeveWebcam.side();

//        signalSleeveWebcam.stopStreaming();

        // drive to preload
        drive.followTrajectorySequenceAsync(preload);
        runThread(scoreReadyThread);

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("cones scored", conesScored);
//            telemetry.addData("current state", trajectoryState);
//            telemetry.addData("canContinue", canContinue);
//            telemetry.addData("canSlidesExtend", canSlidesExtend);
//            telemetry.addData("time", time);
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
                            slides.extendHighAuto();
                            canSlidesExtend = false;
                        }
                        if (time.seconds() >= AutoConstants.DELAY_PRELOAD_PICKUP) {
                            arm.autoScorePos();
                            canContinue = false;
                            canSlidesExtend = true;
                            if (conesScored == 2) {
                                drive.followTrajectorySequenceAsync(coneStackToHighGoalOne);
                            } else if (conesScored == 3) {
                                drive.followTrajectorySequenceAsync(coneStackToHighGoalTwo);
                            } else if (conesScored == 4) {
                                drive.followTrajectorySequenceAsync(coneStackToHighGoalThree);
                            } else if (conesScored >= 5) {
                                drive.followTrajectorySequenceAsync(coneStackToHighGoalFour);
                            } else {
                                drive.followTrajectorySequenceAsync(coneStackToHighGoal);
                            }
                            trajectoryState = TrajectoryState.CS_TO_HG;
                        }
                    }
                    break;

                /** HOW IT WAS BEFORE **/
//                case CS_TO_HG:
//                    if (!drive.isBusy()) {
//                        // maybe lower slides, then open, after 0.15 sec delay?
//                        slides.extendToPosition(slides.getPosition() - 9);
//
//                        clamp.open();
//                        runThread(scoreThread);
//                        if (canContinue) {
//                            conesScored += 1;
//                            slides.extendToPosition(AutoConstants.SLIDE_EXTEND_POSITIONS[conesScored]);
//                            canContinue = false;
//                            canSlidesExtend = true;
//                            time.reset();
//                            drive.followTrajectorySequenceAsync(highGoalToConeStack);
//                            trajectoryState = TrajectoryState.HG_TO_CS;
//                        }
//                    }
//                    break;
                /** END HOW IT WAS BEFORE **/

                case CS_TO_HG:
                    if (!drive.isBusy()) {
                        // maybe lower slides, then open, after 0.15 sec delay?
                        slides.extendToPosition(slides.getPosition() - 14);
                        trajectoryState = TrajectoryState.SCORING;
                        time.reset();
                    }
                    break;
                case SCORING:
                    if (time.seconds() > AutoConstants.DELAY_SCORING) {
                        clamp.open();
                        runThread(scoreThread);
                        if (canContinue) {
                            conesScored += 1;
                            slides.extendToPosition(AutoConstants.SLIDE_EXTEND_POSITIONS[conesScored]);
                            canContinue = false;
                            canSlidesExtend = true;
                            time.reset();
                            if (conesScored == 2) {
                                drive.followTrajectorySequenceAsync(highGoalToConeStackOne);
                            } else if (conesScored == 3) {
                                drive.followTrajectorySequenceAsync(highGoalToConeStackTwo);
                            } else if (conesScored == 4) {
                                drive.followTrajectorySequenceAsync(highGoalToConeStackThree);
                            } else if (conesScored >= 5) {
                                drive.followTrajectorySequenceAsync(highGoalToConeStackFour);
                            } else {
                                drive.followTrajectorySequenceAsync(highGoalToConeStack);
                            }
                            trajectoryState = TrajectoryState.HG_TO_CS;
                        }
                    }
                    break;
                case HG_TO_CS:
                    if (conesScored >= CONE_COUNT) {
//                        slides.rest();
//                        arm.intakePos();
//                        clamp.open();
                        trajectoryState = TrajectoryState.PARK;
                    } else {
                        if (!drive.isBusy()) {
                            clamp.close();
                            if (canSlidesExtend) {
                                slides.extendHighAuto();
                                canSlidesExtend = false;
                            }
                            if (time.seconds() >= AutoConstants.DELAY_PICKUP) {
                                arm.autoScorePos();
                                canContinue = false;
                                canSlidesExtend = true;
                                if (conesScored == 2) {
                                    drive.followTrajectorySequenceAsync(coneStackToHighGoalOne);
                                } else if (conesScored == 3) {
                                    drive.followTrajectorySequenceAsync(coneStackToHighGoalTwo);
                                } else if (conesScored == 4) {
                                    drive.followTrajectorySequenceAsync(coneStackToHighGoalThree);
                                } else if (conesScored >= 5) {
                                    drive.followTrajectorySequenceAsync(coneStackToHighGoalFour);
                                } else {
                                    drive.followTrajectorySequenceAsync(coneStackToHighGoal);
                                }
                                trajectoryState = TrajectoryState.CS_TO_HG;
                            }
                        }
                    }
                    break;
                case PARK:
                    slides.rest();
                    arm.intakePos();
                    clamp.open();
                    switch (parkSide) {
                        case THREE:
                            if (!drive.isBusy()) {
                                trajectoryState = TrajectoryState.IDLE;
                                time.reset();
                            }
                            break;
                        case TWO:
                            if (!drive.isBusy()) {
                                trajectoryState = TrajectoryState.IDLE;
                                drive.followTrajectorySequenceAsync(toMiddlePark);
                                time.reset();
                            }
                            break;
                        case ONE:
                            if (!drive.isBusy()) {
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
                    clamp.open();
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
