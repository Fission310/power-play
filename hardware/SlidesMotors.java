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

    public static double MAX_VEL = 100;
    public static double MAX_ACCEL = 100;
    public static double RETRACTION_MULTIPLIER = 0.75;

    public static double kP = 0.26;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static double POS_REST = -1;
    public static double POS_GROUND = 0;
    public static double POS_PREP_ARM = 15;
    public static double POS_LOW = 20.2;
    public static double POS_MEDIUM = 39;
    public static double POS_HIGH = 58.4;
    public static double POS_HIGH_AUTO = 58.9;

    public static double TELE_DROP_AMT = 4;
    private static double TELE_REST_POS = 0;

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

    public void setMotionConstraints(double value) {
        leftSlideMotor.setMotionConstraints(value, value);
        rightSlideMotor.setMotionConstraints(value, value);
    }

    public void rest() {
        leftSlideMotor.setTargetPosition(POS_REST);
        rightSlideMotor.setTargetPosition(POS_REST);
    }

    public void setTeleRestPos(double pos) {
        TELE_REST_POS = pos;
    }
    public double getTeleRestPos() { return TELE_REST_POS; }

    public void teleRest() {
        leftSlideMotor.setTargetPosition(TELE_REST_POS);
        rightSlideMotor.setTargetPosition(TELE_REST_POS);
    }

    public void extendPrepArm() {
        leftSlideMotor.setTargetPosition(POS_PREP_ARM);
        rightSlideMotor.setTargetPosition(POS_PREP_ARM);
    }

    public void extendGround() {
        leftSlideMotor.setTargetPosition(POS_GROUND);
        rightSlideMotor.setTargetPosition(POS_GROUND);
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

    public void extendHighAuto() {
        leftSlideMotor.setTargetPosition(POS_HIGH_AUTO);
        rightSlideMotor.setTargetPosition(POS_HIGH_AUTO);
    }

    public void extendToPosition(double pos) {
        leftSlideMotor.setTargetPosition(pos);
        rightSlideMotor.setTargetPosition(pos);
    }

    public void ascendABit() {
        leftSlideMotor.setTargetPosition(leftSlideMotor.getPosition() + 9);
        rightSlideMotor.setTargetPosition(rightSlideMotor.getPosition() + 9);
    }

    public void descendABit() {
        leftSlideMotor.setTargetPosition(leftSlideMotor.getPosition() - 3);
        rightSlideMotor.setTargetPosition(rightSlideMotor.getPosition() - 3);
    }

    public void setPower(double power) {
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }

    public void update() {
        leftSlideMotor.update();
        rightSlideMotor.update();
    }

    public double getPosition() {
        return (leftSlideMotor.getPosition() + rightSlideMotor.getPosition()) / 2.0;
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
