package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.motion.MotionProfiledDcMotor;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SlidesMotors extends Mechanism {

    private MotionProfiledDcMotor leftSlideMotor;
    private MotionProfiledDcMotor rightSlideMotor;

    private static final double WHEEL_RADIUS = 1.37795; //inches
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 145.1;

    public static double MAX_VEL = 20;
    public static double MAX_ACCEL = 20;
    public static double RETRACTION_MULTIPLIER = 0.2;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static double POS_LOW = 1;
    public static double POS_MEDIUM = 2;
    public static double POS_HIGH = 3;

    public SlidesMotors(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        leftSlideMotor = new MotionProfiledDcMotor(hwMap, "leftSlideMotor");
        leftSlideMotor.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        leftSlideMotor.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        leftSlideMotor.setRetractionMultiplier(0.2);
        leftSlideMotor.setPIDCoefficients(kP, kI, kD, kF);
        leftSlideMotor.setTargetPosition(0);
        // TODO: uncomment to reverse motor direction
//        leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightSlideMotor = new MotionProfiledDcMotor(hwMap, "rightSlideMotor");
        rightSlideMotor.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        rightSlideMotor.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        rightSlideMotor.setRetractionMultiplier(0.2);
        rightSlideMotor.setPIDCoefficients(kP, kI, kD, kF);
        rightSlideMotor.setTargetPosition(0);
        // TODO: uncomment to reverse motor direction
//        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void rest() {
        leftSlideMotor.setTargetPosition(0);
        rightSlideMotor.setTargetPosition(0);
    }

    public void extendLow() {
        leftSlideMotor.setTargetPosition(POS_LOW);
        rightSlideMotor.setTargetPosition(POS_LOW);
    }

    public void extendMedium() {
        leftSlideMotor.setTargetPosition(POS_MEDIUM);
        rightSlideMotor.setTargetPosition(POS_MEDIUM);
    }

    public void extendHigh() {
        leftSlideMotor.setTargetPosition(POS_HIGH);
        rightSlideMotor.setTargetPosition(POS_HIGH);
    }

    public void update() {
        leftSlideMotor.update();
        rightSlideMotor.update();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("positionLeft", leftSlideMotor.getPosition());
        telemetry.addData("positionRight", rightSlideMotor.getPosition());

        telemetry.addData("velocityLeft", leftSlideMotor.getVelocity());
        telemetry.addData("velocityRight", rightSlideMotor.getVelocity());
    }
}
