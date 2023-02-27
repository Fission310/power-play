package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->
 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class FilteredTwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.6889764; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -5.33; // X is the up and down direction
    public static double PARALLEL_Y = -3.62; // Y is the strafe direction

    public static double PERPENDICULAR_X = -3.7;
    public static double PERPENDICULAR_Y = 2.26;

    public static double X_MULTIPLIER = 1.01785777727;// Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.00481560895;// Multiplier in the Y direction

    // Filter Testing
    public static double GAIN = 0.8;
    LowPassFilter parallelPositionFilter = new LowPassFilter(GAIN);
    LowPassFilter perpendicularPositionFilter = new LowPassFilter(GAIN);

    LowPassFilter parallelVelocityFilter = new LowPassFilter(GAIN);
    LowPassFilter perpendicularVelocityFilter = new LowPassFilter(GAIN);

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    private SampleMecanumDrive drive;

    public FilteredTwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelOdo"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpOdo"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                parallelPositionFilter.estimate(encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER),
                perpendicularPositionFilter.estimate(encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                parallelVelocityFilter.estimate(encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER),
                perpendicularVelocityFilter.estimate(encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER)
        );
    }
}