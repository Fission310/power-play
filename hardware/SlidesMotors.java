package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.motion.MotionProfiledDcMotor;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class SlidesMotors extends Mechanism {

    private MotionProfiledDcMotor leftSlideMotor;
    private MotionProfiledDcMotor rightSlideMotor;

    private static final double WHEEL_RADIUS = 1.37795; //inches
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 145.1;

    public static double MAX_VEL = 50;
    public static double MAX_ACCEL = 50;
    public static double RETRACTION_MULTIPLIER = 0.7;

    public static double kP = 0.1;
    public static double kI = 0;
    public static double kD = 0;
    // TODO: Increase for faster slides
    public static double kF = 0;

    public static double POS_PREP_ARM = 10;
    public static double POS_CONE_STACK = 20;
    public static double POS_LOW = 33;
    public static double POS_MEDIUM = 55;
    public static double POS_HIGH = 68;

    public SlidesMotors(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        leftSlideMotor = new MotionProfiledDcMotor(hwMap, "leftSlideMotor");
        leftSlideMotor.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        leftSlideMotor.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        leftSlideMotor.setRetractionMultiplier(RETRACTION_MULTIPLIER);
        leftSlideMotor.setPIDCoefficients(kP, kI, kD, kF);
        leftSlideMotor.setTargetPosition(0);
        // TODO: uncomment to reverse motor direction
//        leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightSlideMotor = new MotionProfiledDcMotor(hwMap, "rightSlideMotor");
        rightSlideMotor.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        rightSlideMotor.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        rightSlideMotor.setRetractionMultiplier(RETRACTION_MULTIPLIER);
        rightSlideMotor.setPIDCoefficients(kP, kI, kD, kF);
        rightSlideMotor.setTargetPosition(0);
        // TODO: uncomment to reverse motor direction
        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void rest() {
        leftSlideMotor.setTargetPosition(0);
        rightSlideMotor.setTargetPosition(0);
    }

    public void extendPrepArm() {
        leftSlideMotor.setTargetPosition(POS_PREP_ARM);
        rightSlideMotor.setTargetPosition(POS_PREP_ARM);
    }

    public void extendConeStack() {
        leftSlideMotor.setTargetPosition(POS_CONE_STACK);
        rightSlideMotor.setTargetPosition(POS_CONE_STACK);
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

    public void descendABit() {
        leftSlideMotor.setTargetPosition(leftSlideMotor.getPosition() - 3);
        rightSlideMotor.setTargetPosition(rightSlideMotor.getPosition() - 3);
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

        telemetry.addData("currentLeft", leftSlideMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("currentRight", rightSlideMotor.getCurrent(CurrentUnit.AMPS));
    }
}
