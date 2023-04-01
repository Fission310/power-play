package org.firstinspires.ftc.teamcode.opmode.auton.highgoal;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.stuyfission.fissionlib.command.AutoCommandMachine;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Clamp;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.SignalSleeveWebcam;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.SlidesMotors;
import org.firstinspires.ftc.teamcode.opmode.auton.AutoConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@SuppressWarnings("FieldMayBeFinal")
@Autonomous (name = "HIGH COMMAND 6 Cone Auto", group = "_ared", preselectTeleOp = "Drift Comp Main")
public class CommandBasedSixConeAuto extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Arm arm;
    private SlidesMotors slides;
    private Clamp clamp;
    private final SignalSleeveWebcam signalSleeveWebcam = new SignalSleeveWebcam(this, "rightWebcam", SignalSleeveWebcam.ROBOT_SIDE.CONTROL_HUB);

    private TrajectorySequence preload;
    private TrajectorySequence preloadToConeStack;
    private TrajectorySequence coneStackToHighGoal;
    private TrajectorySequence highGoalToConeStack;
    private TrajectorySequence toParkTemp;
    private TrajectorySequence toLeftPark;
    private TrajectorySequence toMiddlePark;
    private TrajectorySequence toRightPark;


    private Command preloadTraj = () -> drive.followTrajectorySequenceAsync(preload);
    private Command highSlides = () -> slides.extendHighAuto();
    private Command armScorePos = () -> arm.autoScorePos();
    private Command preloadToConeStackTraj = () -> drive.followTrajectorySequenceAsync(preloadToConeStack);
    private CommandSequence driveToPreloadCommandSeq = new CommandSequence()
            .addCommand(preloadTraj)
            .addWaitCommand(0.5)
            .addCommand(highSlides)
            .addWaitCommand(0.1)
            .addCommand(armScorePos)
            .build();

    private Command closeClamp = () -> clamp.close();
    private Command coneStackToHighGoalTraj = () -> drive.followTrajectorySequenceAsync(coneStackToHighGoal);
    private CommandSequence pickupConeAndDriveToHighGoalCommandSeq = new CommandSequence()
            .addCommand(closeClamp)
            .addWaitCommand(0.1)
            .addCommand(highSlides)
            .addWaitCommand(0.1)
            .addCommand(armScorePos)
            .addCommand(coneStackToHighGoalTraj)
            .build();

    private Command dropSlides = () -> slides.extendToPosition(slides.getPosition() - 3);
    private Command openClamp = () -> clamp.open();
    private Command resetArmSlides = () -> {
        conesScored++;
        arm.moveToPos(AutoConstants.ARM_CONE_STACK_POSITIONS[conesScored]);
        slides.extendToPosition(AutoConstants.SLIDE_EXTEND_POSITIONS[conesScored]);
    };
    private Command highGoalToConeStackTraj = () -> drive.followTrajectorySequenceAsync(highGoalToConeStack);
    private Command intakePosClamp = () -> clamp.intakePos();
    private CommandSequence scoreAndDriveToConeStackCommandSeq = new CommandSequence()
            .addCommand(dropSlides)
            .addWaitCommand(0.05)
            .addCommand(openClamp)
            .addWaitCommand(0.1)
            .addCommand(resetArmSlides)
            .addCommand(highGoalToConeStackTraj)
            .addWaitCommand(0.35)
            .addCommand(intakePosClamp)
            .build();

    private CommandSequence scorePreloadAndDriveToConeStackCommandSeq = new CommandSequence()
            .addCommand(dropSlides)
            .addWaitCommand(0.05)
            .addCommand(openClamp)
            .addCommand(resetArmSlides)
            .addCommand(preloadToConeStackTraj)
            .addWaitCommand(0.35)
            .addCommand(intakePosClamp)
            .build();

    private AutoCommandMachine autoCommandMachine = new AutoCommandMachine()
            .addCommandSequence(driveToPreloadCommandSeq)
            .addCommandSequence(scorePreloadAndDriveToConeStackCommandSeq)
            .addCommandSequence(pickupConeAndDriveToHighGoalCommandSeq)
            .addCommandSequence(scoreAndDriveToConeStackCommandSeq)
            .build();

    /** VERY IMPORTANT **/
    private static int conesScored;

    private static final TrajectoryVelocityConstraint VELO = SampleMecanumDrive.getVelocityConstraint(43, Math.toRadians(250), Math.toRadians(250));
    private static final TrajectoryAccelerationConstraint ACCEL = SampleMecanumDrive.getAccelerationConstraint(43);

    private static final TrajectoryVelocityConstraint FAST_VELO = SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(250), Math.toRadians(250));
    private static final TrajectoryAccelerationConstraint FAST_ACCEL = SampleMecanumDrive.getAccelerationConstraint(52);

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new Arm(this);
        slides = new SlidesMotors(this);
        clamp = new Clamp(this);

        signalSleeveWebcam.init(hardwareMap);
        clamp.init(hardwareMap);
        arm.init(hardwareMap);
        slides.init(hardwareMap);

        drive.setPoseEstimate(AutoConstants.RR_START_POSE);

        conesScored = 0;

        preload = drive.trajectorySequenceBuilder(AutoConstants.RR_START_POSE)
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .lineToLinearHeading(AutoConstants.RR_ODO_PRELOAD_HIGH_GOAL_POSE_SIX)
                .lineToConstantHeading(AutoConstants.RR_ODO_PRELOAD_HIGH_GOAL_VECTOR_SIX)
                .build();

        preloadToConeStack = drive.trajectorySequenceBuilder(preload.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .lineToLinearHeading(AutoConstants.RR_ODO_PRELOAD_HIGH_GOAL_POSE_SIX)
                .lineToLinearHeading(AutoConstants.RR_ODO_PRELOAD_CONE_STACK_POSE_SIX)
                .setConstraints(VELO, ACCEL)
                .lineToConstantHeading(AutoConstants.RR_ODO_CONE_STACK_VECTOR_SIX)
                .build();

        coneStackToHighGoal = drive.trajectorySequenceBuilder(preloadToConeStack.end())
                .setConstraints(VELO, ACCEL)
                .setReversed(true)
                .setTangent(AutoConstants.RR_HEADING)
                .splineTo(AutoConstants.RR_ODO_HIGH_GOAL_VECTOR_SIX, AutoConstants.RR_ODO_HIGH_GOAL_HEADING)
                .build();

        highGoalToConeStack = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setConstraints(VELO, ACCEL)
                .setReversed(false)
                .setTangent(AutoConstants.RR_ODO_CONE_STACK_TANGENT)
                .splineTo(AutoConstants.RR_ODO_CONE_STACK_VECTOR_SIX, AutoConstants.RR_ODO_CONE_STACK_HEADING)
                .build();

        toParkTemp = drive.trajectorySequenceBuilder(coneStackToHighGoal.end())
                .setConstraints(VELO, ACCEL)
                .setReversed(false)
                .setTangent(AutoConstants.RR_ODO_CONE_STACK_TANGENT)
                .splineTo(AutoConstants.RR_ODO_MIDDLE_PARK_VECTOR, AutoConstants.RR_ODO_MIDDLE_PARK_HEADING)
                .build();

        toLeftPark = drive.trajectorySequenceBuilder(toParkTemp.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .lineToLinearHeading(AutoConstants.RR_ODO_LEFT_PARK_POSE)
                .back(9)
                .build();

        toMiddlePark = drive.trajectorySequenceBuilder(toParkTemp.end())
                .setConstraints(VELO, ACCEL)
                .lineToLinearHeading(new Pose2d(AutoConstants.RR_ODO_MIDDLE_PARK_VECTOR.getX() - 2, AutoConstants.RR_ODO_MIDDLE_PARK_VECTOR.getY(), Math.toRadians(90)))
                .build();

        toRightPark = drive.trajectorySequenceBuilder(toParkTemp.end())
                .setConstraints(FAST_VELO, FAST_ACCEL)
                .lineToLinearHeading(AutoConstants.RR_ODO_RIGHT_PARK_POSE)
                .back(9)
                .build();

        clamp.close();
        arm.intakePos();

        waitForStart();

        SignalSleeveWebcam.Side parkSide = signalSleeveWebcam.side();

        signalSleeveWebcam.stopStreaming();


        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("cones scored", conesScored);
            telemetry.update();
            drive.update();
            slides.update();
            autoCommandMachine.run(drive.isBusy());
        }
    }
}
