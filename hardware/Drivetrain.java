package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Drivetrain extends Mechanism {

    SampleMecanumDrive rrDrive;

    public Drivetrain(LinearOpMode opMode) { this.opMode = opMode; }

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }
    DriveMode driveMode = DriveMode.ROBOT_CENTRIC;

    @Override
    public void init(HardwareMap hwMap) {
        rrDrive = new SampleMecanumDrive(hwMap);
        rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop(Gamepad gamepad) {

        if (gamepad.left_stick_button) {
            driveMode = DriveMode.ROBOT_CENTRIC;
        } else if (gamepad.right_stick_button) {
            driveMode = DriveMode.FIELD_CENTRIC;
        }

        switch (driveMode) {
            case ROBOT_CENTRIC:
                rrDrive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad.left_stick_y,
                                -gamepad.left_stick_x,
                                -gamepad.right_stick_x
                        )
                );
                break;
            case FIELD_CENTRIC:
                Vector2d input = new Vector2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x
                ).rotated(-1 * (rrDrive.getPoseEstimate().getHeading()));

                rrDrive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad.right_stick_x
                        )
                );
                break;
        }

        rrDrive.update();
    }

}
