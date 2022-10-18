package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class RED_cycles extends LinearOpMode {

    public static double WAREHOUSE_WAIT = 0.15;

    public static double WALL_POS = -70.5 + (12.5/2.0);

    private static final Pose2d SCORE_0 = new Pose2d(-11, WALL_POS+1);
    private static final Pose2d SCORE_1 = new Pose2d(-11+1, WALL_POS+1);
    private static final Pose2d SCORE_2 = new Pose2d(-11, WALL_POS+1);
    private static final Pose2d SCORE_3 = new Pose2d(-11+1, WALL_POS+1);
    private static final Pose2d SCORE_4 = new Pose2d(-11+1, WALL_POS+1);
    private static final Pose2d SCORE_5 = new Pose2d(-11+1, WALL_POS+1);

    private static final Pose2d PARK = new Pose2d(41+3, WALL_POS+2);
    private static final Pose2d WAREHOUSE_0 = new Pose2d(44, WALL_POS+1);
    private static final Pose2d WAREHOUSE_1 = new Pose2d(50, WALL_POS+1);
    private static final Pose2d WAREHOUSE_2 = new Pose2d(54, WALL_POS+1);
    private static final Pose2d WAREHOUSE_3 = new Pose2d(55.5, WALL_POS+1);
    private static final Pose2d WAREHOUSE_4 = new Pose2d(57, WALL_POS+1);
    private static final Pose2d WAREHOUSE_5 = new Pose2d(58, WALL_POS+1);

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(18, WALL_POS);

        TrajectorySequence cycles = drive.trajectorySequenceBuilder(startPose)
                // ======== CV ======== //
                .lineToLinearHeading(SCORE_0)
                // ==================== //

                // ======== CYCLE 1 ======== //
                .lineToLinearHeading(WAREHOUSE_0)
                .waitSeconds(WAREHOUSE_WAIT)
                .lineToLinearHeading(SCORE_0)
                // ========================= //

                // ======== CYCLE 2 ======== //
                .lineToLinearHeading(WAREHOUSE_1)
                .waitSeconds(WAREHOUSE_WAIT)
                .lineToLinearHeading(SCORE_1)
                // ========================= //

                // ======== CYCLE 3 ======== //
                .lineToLinearHeading(WAREHOUSE_2)
                .waitSeconds(WAREHOUSE_WAIT)
                .lineToLinearHeading(SCORE_2)
                // ========================= //

                // ======== CYCLE 4 ======== //
                .lineToLinearHeading(WAREHOUSE_3)
                .waitSeconds(WAREHOUSE_WAIT)
                .lineToLinearHeading(SCORE_3)
                // ========================= //

                // ======== CYCLE 5 ======== //
                .lineToLinearHeading(WAREHOUSE_4)
                .waitSeconds(WAREHOUSE_WAIT)
                .lineToLinearHeading(SCORE_4)
                // ========================= //

                // ======== CYCLE 6 ======== //
                .lineToLinearHeading(WAREHOUSE_5)
                .waitSeconds(WAREHOUSE_WAIT)
                .lineToLinearHeading(SCORE_5)
                // ========================= //

                // ======== PARK ======= //
                .lineToLinearHeading(PARK)
                // ===================== //

                .build();

        drive.setPoseEstimate(startPose);

        waitForStart();

        drive.followTrajectorySequenceAsync(cycles);

        while(opModeIsActive()) {
            drive.update();
        }
    }
}
