package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.FreightSensor;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Webcam;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Webcam.Location;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.slides.SlideMechanism;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "_blue")
@Config
public class BLUE_cyclesCancelable extends LinearOpMode {

    private static double HEADING = Math.toRadians(180);

    public static double WAREHOUSE_WAIT = 0.15;

    public static double WALL_POS = 70.5 - (12.15/2.0);

    private static final Pose2d SCORE_0 = new Pose2d(-11.5+3.6, WALL_POS+0.6, HEADING);
    private static final Pose2d SCORE_1 = new Pose2d(-11.5+4.6, WALL_POS+1.6, HEADING);
    private static final Pose2d SCORE_2 = new Pose2d(-11.5+4.6, WALL_POS+1.6, HEADING);
    private static final Pose2d SCORE_3 = new Pose2d(-11.5+5.2, WALL_POS+2.6, HEADING);
    private static final Pose2d SCORE_4 = new Pose2d(-11.5+5.6, WALL_POS+3.6, HEADING);
    private static final Pose2d SCORE_5 = new Pose2d(-11.5+6, WALL_POS+4.6, HEADING);


    private static final Pose2d WAREHOUSE_0 = new Pose2d(47, WALL_POS+0.6, HEADING);
    private static final Pose2d WAREHOUSE_1 = new Pose2d(50, WALL_POS+1.6, HEADING);
    private static final Pose2d WAREHOUSE_2 = new Pose2d(53, WALL_POS+1.6, HEADING);
    private static final Pose2d WAREHOUSE_3 = new Pose2d(56, WALL_POS+2.6, HEADING);
    private static final Pose2d WAREHOUSE_4 = new Pose2d(59, WALL_POS+3.6, HEADING);
    private static final Pose2d WAREHOUSE_5 = new Pose2d(59, WALL_POS+4.6, HEADING);

    private static final Pose2d PARK = new Pose2d(50, WALL_POS+4.6, HEADING);

    private TrajectorySequence cv;
    private TrajectorySequence wh0;
    private TrajectorySequence sc0;
    private TrajectorySequence wh1;
    private TrajectorySequence sc1;
    private TrajectorySequence wh2;
    private TrajectorySequence sc2;
    private TrajectorySequence wh3;
    private TrajectorySequence sc3;
    private TrajectorySequence wh4;
    private TrajectorySequence sc4;
    private TrajectorySequence wh5;
    private TrajectorySequence sc5;

    private TrajectorySequence park;

    private enum TrajState {
        CV,
        SC_CV,
        WH_0,
        SC_0,
        WH_1,
        SC_1,
        WH_2,
        SC_2,
        WH_3,
        SC_3,
        WH_4,
        SC_4,
        WH_5,
        SC_5,
        IDLE
    }
    TrajState currentTraj;
    Slides.SlidesState slidesState;

    ElapsedTime time = new ElapsedTime();
    public static double TIP_WAIT = 2;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FreightSensor sensor = new FreightSensor(this);
        SlideMechanism slides = new SlideMechanism(this);
        Acquirer acquirer = new Acquirer(this);
        Webcam webcam = new Webcam(this);
        FreightSensor freightSensor = new FreightSensor(this);

        sensor.init(hardwareMap);
        slides.init(hardwareMap);
        webcam.init(hardwareMap);
        acquirer.init(hardwareMap);
        freightSensor.init(hardwareMap);

        Pose2d startPose = new Pose2d(12, WALL_POS, HEADING);

        cv = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(SCORE_0)

                .build();

        wh0 = drive.trajectorySequenceBuilder(cv.end())
                .lineToLinearHeading(WAREHOUSE_0)
                .waitSeconds(WAREHOUSE_WAIT)

                .build();

        sc0 = drive.trajectorySequenceBuilder(wh0.end())
                .lineToLinearHeading(SCORE_0)
                .build();

        wh1 = drive.trajectorySequenceBuilder(sc0.end())
                .lineToLinearHeading(WAREHOUSE_1)
                .waitSeconds(WAREHOUSE_WAIT)

                .build();

        sc1 = drive.trajectorySequenceBuilder(wh1.end())
                .lineToLinearHeading(SCORE_1)

                .build();

        wh2 = drive.trajectorySequenceBuilder(sc1.end())
                .lineToLinearHeading(WAREHOUSE_2)
                .waitSeconds(WAREHOUSE_WAIT)

                .build();

        sc2 = drive.trajectorySequenceBuilder(wh2.end())
                .lineToLinearHeading(SCORE_2)

                .build();

        wh3 = drive.trajectorySequenceBuilder(sc2.end())
                .lineToLinearHeading(WAREHOUSE_3)
                .waitSeconds(WAREHOUSE_WAIT)

                .build();

        sc3 = drive.trajectorySequenceBuilder(wh3.end())
                .lineToLinearHeading(SCORE_3)

                .build();

        wh4 = drive.trajectorySequenceBuilder(sc3.end())
                .lineToLinearHeading(WAREHOUSE_4)
                .waitSeconds(WAREHOUSE_WAIT)

                .build();

        sc4 = drive.trajectorySequenceBuilder(wh4.end())
                .lineToLinearHeading(SCORE_4)

                .build();

        wh5 = drive.trajectorySequenceBuilder(sc4.end())
                .lineToLinearHeading(WAREHOUSE_5)
                .waitSeconds(WAREHOUSE_WAIT)

                .build();

        sc5 = drive.trajectorySequenceBuilder(wh5.end())
                .lineToLinearHeading(SCORE_5)

                .build();


        park = drive.trajectorySequenceBuilder(sc1.end())
                .lineToLinearHeading(PARK)

                .build();

        drive.setPoseEstimate(startPose);

        currentTraj = TrajState.CV;
        slidesState = Slides.SlidesState.WAIT;
        slides.close();

        waitForStart();

        Location location = webcam.location();
        webcam.stopStreaming();

        acquirer.intakeLeft();
        acquirer.intakeRight();
        acquirer.acquirerState = Acquirer.AcquirerState.ACQUIRER_START_LEFT;

        drive.followTrajectorySequenceAsync(cv);

        while(opModeIsActive() && !isStopRequested()) {

            switch(currentTraj) {
                case CV:
                    if (!drive.isBusy()) {
                        currentTraj = TrajState.SC_CV;
                    }
                    break;
                case SC_CV:
                    switch (slidesState) {
                        case WAIT:
                            switch (location) {
                                case LEFT:
                                    slides.extendLevel1TEMP();
                                    slides.close();

                                    time.reset();
                                    slidesState = Slides.SlidesState.DELAY;
                                    break;
                                case MIDDLE:
                                    slides.extendLevel2TEMP();
                                    slides.close();

                                    time.reset();
                                    slidesState = Slides.SlidesState.DELAY;
                                    break;
                                case RIGHT:
                                    slides.extendLevel3();
                                    slides.close();

                                    time.reset();
                                    slidesState = Slides.SlidesState.DELAY;
                                    break;
                            }
                            break;
                        case DELAY:
                            switch (location) {
                                case LEFT:
                                    if (time.seconds() > Slides.LEVEL1_ARM_TEMP_WAIT) {
                                        slides.armLevel1();
                                    }
                                    if (time.seconds() > Slides.LEVEL1_TEMP_WAIT) {
                                        slides.extendLevel1();

                                        slidesState = Slides.SlidesState.TIP;
                                    }
                                    break;
                                case MIDDLE:
                                    if (time.seconds() > Slides.LEVEL1_ARM_TEMP_WAIT) {
                                        slides.armLevel2();
                                    }
                                    if (time.seconds() > Slides.LEVEL2_TEMP_WAIT) {
                                        slides.extendLevel2();

                                        slidesState = Slides.SlidesState.TIP;
                                    }
                                    break;
                                case RIGHT:
                                    if (time.seconds() > Slides.LEVEL3_TEMP_WAIT) {
                                        slides.armLevel3();

                                        slidesState = Slides.SlidesState.TIP;
                                    }
                                    break;
                            }
                            break;
                        case TIP:
                            if (time.seconds() > TIP_WAIT) {
                                slides.open();

                                time.reset();
                                slidesState = Slides.SlidesState.TEMP_RETRACT;
                            }
                            break;
                        case TEMP_RETRACT:

                            switch (location) {
                                case LEFT:
                                case MIDDLE:
                                    if (time.seconds() > Slides.TEMP_RETRACT_WAIT) {
                                        slides.restTEMP();
                                        slidesState = Slides.SlidesState.TEMP_CARRIAGE;
                                        time.reset();
                                    }
                                    break;
                                case RIGHT:
                                    if (time.seconds() > Slides.LEVEL_3_TEMP_RETRACT_WAIT) {
                                        slidesState = Slides.SlidesState.TEMP_CARRIAGE;
                                        time.reset();
                                    }
                                    break;

                            }
                            break;
                        case TEMP_CARRIAGE:
                            if (time.seconds() > Slides.TEMP_CARRIAGE_WAIT) {
                                slides.restCarriage();
                                time.reset();
                                slidesState = Slides.SlidesState.TIP_DELAY;
                            }
                            break;
                        case TIP_DELAY:
                            switch (location) {
                                case LEFT:
                                    if (time.seconds() > Slides.LEVEL1_TIP_WAIT) {
                                        time.reset();
                                        slides.rest();
                                        currentTraj = TrajState.WH_0;
                                        drive.followTrajectorySequenceAsync(wh0);
                                    }
                                    break;
                                case MIDDLE:
                                    if (time.seconds() > Slides.LEVEL2_TIP_WAIT) {
                                        time.reset();
                                        slides.rest();
                                        currentTraj = TrajState.WH_0;
                                        drive.followTrajectorySequenceAsync(wh0);
                                    }
                                    break;
                                case RIGHT:
                                    if (time.seconds() > Slides.LEVEL3_TIP_WAIT) {
                                        time.reset();
                                        slides.rest();
                                        currentTraj = TrajState.WH_0;
                                        drive.followTrajectorySequenceAsync(wh0);
                                    }
                                    break;
                            }
                            break;
                    }
                    break;
                case WH_0:
                    if (sensor.hasFreightRight()) {
                        drive.breakFollowing();
                        drive.setDrivePower(new Pose2d());
                    }
                    if (!drive.isBusy()) {
                        slidesState = Slides.SlidesState.WAIT;
                        currentTraj = TrajState.SC_0;
                        drive.followTrajectorySequenceAsync(sc0);
                    }
                    break;

                case SC_0:
                    if (!drive.isBusy()) {
                        // score level 3
                        switch (slidesState) {
                            case WAIT:
                                slides.extendLevel3();
                                slides.close();

                                time.reset();
                                slidesState = Slides.SlidesState.DELAY;
                                break;
                            case DELAY:
                                if (time.seconds() > Slides.LEVEL3_TEMP_WAIT) {
                                    slides.armLevel3();

                                    slidesState = Slides.SlidesState.TIP;
                                }
                                break;
                            case TIP:
                                if (time.seconds() > TIP_WAIT) {
                                    slides.open();

                                    time.reset();
                                    slidesState = Slides.SlidesState.TEMP_RETRACT;
                                }
                                break;
                            case TEMP_RETRACT:
                                if (time.seconds() > Slides.TEMP_RETRACT_WAIT) {
                                    time.reset();
                                    slidesState = Slides.SlidesState.TEMP_CARRIAGE;
                                    time.reset();
                                    break;
                                }
                                break;
                            case TEMP_CARRIAGE:
                                if (time.seconds() > Slides.TEMP_CARRIAGE_WAIT) {
                                    slides.restCarriage();
                                    time.reset();
                                    slidesState = Slides.SlidesState.TIP_DELAY;
                                }
                                break;
                            case TIP_DELAY:
                                if (time.seconds() > Slides.LEVEL3_TIP_WAIT) {
                                    time.reset();
                                    slides.rest();
                                    currentTraj = TrajState.WH_1;
                                    drive.followTrajectorySequenceAsync(wh1);
                                }
                                break;
                        }
                    }
                    break;
                case WH_1:
                    if (sensor.hasFreightRight()) {
                        drive.breakFollowing();
                        drive.setDrivePower(new Pose2d());
                    }
                    if (!drive.isBusy()) {
                        slidesState = Slides.SlidesState.WAIT;
                        currentTraj = TrajState.SC_1;
                        drive.followTrajectorySequenceAsync(sc1);
                    }
                    break;
                case SC_1:
                    if (!drive.isBusy()) {
                        // score level 3
                        switch (slidesState) {
                            case WAIT:
                                slides.extendLevel3();
                                slides.close();

                                time.reset();
                                slidesState = Slides.SlidesState.DELAY;
                                break;
                            case DELAY:
                                if (time.seconds() > Slides.LEVEL3_TEMP_WAIT) {
                                    slides.armLevel3();

                                    slidesState = Slides.SlidesState.TIP;
                                }
                                break;
                            case TIP:
                                if (time.seconds() > TIP_WAIT) {
                                    slides.open();

                                    time.reset();
                                    slidesState = Slides.SlidesState.TEMP_RETRACT;
                                }
                                break;
                            case TEMP_RETRACT:
                                if (time.seconds() > Slides.LEVEL_3_TEMP_RETRACT_WAIT) {
                                    time.reset();
                                    slidesState = Slides.SlidesState.TEMP_CARRIAGE;
                                    time.reset();
                                    break;
                                }
                                break;
                            case TEMP_CARRIAGE:
                                if (time.seconds() > Slides.TEMP_CARRIAGE_WAIT) {
                                    slides.restCarriage();
                                    time.reset();
                                    slidesState = Slides.SlidesState.TIP_DELAY;
                                }
                                break;
                            case TIP_DELAY:
                                if (time.seconds() > Slides.LEVEL3_TIP_WAIT) {
                                    time.reset();
                                    slides.rest();
                                    currentTraj = TrajState.WH_2;
                                    drive.followTrajectorySequenceAsync(wh2);
                                }
                                break;
                        }
                    }
                    break;
                case WH_2:
                    if (sensor.hasFreightRight()) {
                        drive.breakFollowing();
                        drive.setDrivePower(new Pose2d());
                    }
                    if (!drive.isBusy()) {
                        slidesState = Slides.SlidesState.WAIT;
                        currentTraj = TrajState.SC_2;
                        drive.followTrajectorySequenceAsync(sc2);
                    }
                    break;
                case SC_2:
                    if (!drive.isBusy()) {
                        // score level 3
                        switch (slidesState) {
                            case WAIT:
                                slides.extendLevel3();
                                slides.close();

                                time.reset();
                                slidesState = Slides.SlidesState.DELAY;
                                break;
                            case DELAY:
                                if (time.seconds() > Slides.LEVEL3_TEMP_WAIT) {
                                    slides.armLevel3();

                                    slidesState = Slides.SlidesState.TIP;
                                }
                                break;
                            case TIP:
                                if (time.seconds() > TIP_WAIT) {
                                    slides.open();

                                    time.reset();
                                    slidesState = Slides.SlidesState.TEMP_RETRACT;
                                }
                                break;
                            case TEMP_RETRACT:
                                if (time.seconds() > Slides.LEVEL_3_TEMP_RETRACT_WAIT) {
                                    time.reset();
                                    slidesState = Slides.SlidesState.TEMP_CARRIAGE;
                                    time.reset();
                                    break;
                                }
                                break;
                            case TEMP_CARRIAGE:
                                if (time.seconds() > Slides.TEMP_CARRIAGE_WAIT) {
                                    slides.restCarriage();
                                    time.reset();
                                    slidesState = Slides.SlidesState.TIP_DELAY;
                                }
                                break;
                            case TIP_DELAY:
                                if (time.seconds() > Slides.LEVEL3_TIP_WAIT) {
                                    time.reset();
                                    slides.rest();
                                    currentTraj = TrajState.WH_3;
                                    drive.followTrajectorySequenceAsync(wh3);
                                }
                                break;
                        }
                    }
                    break;
                case WH_3:
                    if (sensor.hasFreightRight()) {
                        drive.breakFollowing();
                        drive.setDrivePower(new Pose2d());
                    }
                    if (!drive.isBusy()) {
                        slidesState = Slides.SlidesState.WAIT;
                        currentTraj = TrajState.SC_3;
                        drive.followTrajectorySequenceAsync(sc3);
                    }
                    break;
                case SC_3:
                    if (!drive.isBusy()) {
                        // score level 3
                        switch (slidesState) {
                            case WAIT:
                                slides.extendLevel3();
                                slides.close();

                                time.reset();
                                slidesState = Slides.SlidesState.DELAY;
                                break;
                            case DELAY:
                                if (time.seconds() > Slides.LEVEL3_TEMP_WAIT) {
                                    slides.armLevel3();

                                    slidesState = Slides.SlidesState.TIP;
                                }
                                break;
                            case TIP:
                                if (time.seconds() > TIP_WAIT) {
                                    slides.open();

                                    time.reset();
                                    slidesState = Slides.SlidesState.TEMP_RETRACT;
                                }
                                break;
                            case TEMP_RETRACT:
                                if (time.seconds() > Slides.LEVEL_3_TEMP_RETRACT_WAIT) {
                                    time.reset();
                                    slidesState = Slides.SlidesState.TEMP_CARRIAGE;
                                    time.reset();
                                    break;
                                }
                                break;
                            case TEMP_CARRIAGE:
                                if (time.seconds() > Slides.TEMP_CARRIAGE_WAIT) {
                                    slides.restCarriage();
                                    time.reset();
                                    slidesState = Slides.SlidesState.TIP_DELAY;
                                }
                                break;
                            case TIP_DELAY:
                                if (time.seconds() > Slides.LEVEL3_TIP_WAIT) {
                                    time.reset();
                                    slides.rest();
                                    currentTraj = TrajState.WH_4;
                                    drive.followTrajectorySequenceAsync(wh4);
                                }
                                break;
                        }
                    }
                    break;
                case WH_4:
                    if (sensor.hasFreightRight()) {
                        drive.breakFollowing();
                        drive.setDrivePower(new Pose2d());
                    }
                    if (!drive.isBusy()) {
                        slidesState = Slides.SlidesState.WAIT;
                        currentTraj = TrajState.SC_4;
                        drive.followTrajectorySequenceAsync(sc4);
                    }
                    break;
                case SC_4:
                    if (!drive.isBusy()) {
                        // score level 3
                        switch (slidesState) {
                            case WAIT:
                                slides.extendLevel3();
                                slides.close();

                                time.reset();
                                slidesState = Slides.SlidesState.DELAY;
                                break;
                            case DELAY:
                                if (time.seconds() > Slides.LEVEL3_TEMP_WAIT) {
                                    slides.armLevel3();

                                    slidesState = Slides.SlidesState.TIP;
                                }
                                break;
                            case TIP:
                                if (time.seconds() > TIP_WAIT) {
                                    slides.open();

                                    time.reset();
                                    slidesState = Slides.SlidesState.TEMP_RETRACT;
                                }
                                break;
                            case TEMP_RETRACT:
                                if (time.seconds() > Slides.LEVEL_3_TEMP_RETRACT_WAIT) {
                                    time.reset();
                                    slidesState = Slides.SlidesState.TEMP_CARRIAGE;
                                    time.reset();
                                    break;
                                }
                                break;
                            case TEMP_CARRIAGE:
                                if (time.seconds() > Slides.TEMP_CARRIAGE_WAIT) {
                                    slides.restCarriage();
                                    time.reset();
                                    slidesState = Slides.SlidesState.TIP_DELAY;
                                }
                                break;
                            case TIP_DELAY:
                                if (time.seconds() > Slides.LEVEL3_TIP_WAIT) {
                                    time.reset();
                                    slides.rest();
                                    currentTraj = TrajState.WH_5;
                                    drive.followTrajectorySequenceAsync(wh5);
                                }
                                break;
                        }
                    }
                    break;
                case WH_5:
                    if (sensor.hasFreightRight()) {
                        drive.breakFollowing();
                        drive.setDrivePower(new Pose2d());
                    }
                    if (!drive.isBusy()) {
                        slidesState = Slides.SlidesState.WAIT;
                        currentTraj = TrajState.SC_5;
                        drive.followTrajectorySequenceAsync(sc5);
                    }
                    break;
                case SC_5:
                    if (!drive.isBusy()) {
                        // score level 3
                        switch (slidesState) {
                            case WAIT:
                                slides.extendLevel3();
                                slides.close();

                                time.reset();
                                slidesState = Slides.SlidesState.DELAY;
                                break;
                            case DELAY:
                                if (time.seconds() > Slides.LEVEL3_TEMP_WAIT) {
                                    slides.armLevel3();

                                    slidesState = Slides.SlidesState.TIP;
                                }
                                break;
                            case TIP:
                                if (time.seconds() > TIP_WAIT) {
                                    slides.open();

                                    time.reset();
                                    slidesState = Slides.SlidesState.TEMP_RETRACT;
                                }
                                break;
                            case TEMP_RETRACT:
                                if (time.seconds() > Slides.LEVEL_3_TEMP_RETRACT_WAIT) {
                                    time.reset();
                                    slidesState = Slides.SlidesState.TEMP_CARRIAGE;
                                    time.reset();
                                    break;
                                }
                                break;
                            case TEMP_CARRIAGE:
                                if (time.seconds() > Slides.TEMP_CARRIAGE_WAIT) {
                                    slides.restCarriage();
                                    time.reset();
                                    slidesState = Slides.SlidesState.TIP_DELAY;
                                }
                                break;
                            case TIP_DELAY:
                                if (time.seconds() > Slides.LEVEL3_TIP_WAIT) {
                                    time.reset();
                                    slides.rest();
                                    currentTraj = TrajState.IDLE;
                                    drive.followTrajectorySequenceAsync(park);
                                }
                                break;
                        }
                    }
                    break;
                case IDLE:
                    break;
            }

            telemetry.addData("Traj state", currentTraj);
            telemetry.addData("slides state", slidesState);
            telemetry.addData("is drive busy", drive.isBusy());

            acquirer.telemetry(telemetry);

            telemetry.update();

            slides.update();
            drive.update();
            acquirer.autonLoop();

//            freightSensor.telemetry(telemetry);
        }
    }
}
