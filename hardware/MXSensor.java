package org.firstinspires.ftc.teamcode.hardware;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Class for Ultrasonic Analog Sensors
@Config
public class MXSensor extends Mechanism {

    public enum Side {
        LEFT,
        RIGHT,
        ALL
    }
    Side side;

    /* TODO: sensor pcp -> hub
        black - ground -> ground
        red - V+ (next to ground) -> 3.3V
        white - nothing
        blue - 3 -> 0
    */

    // Analog Sensors, they report voltage
    private AnalogInput mxSensorLeft;
    private AnalogInput mxSensorRight;



    // rev hubs supply 3.3 volts to analog ports
    double SUPPLIED_VOLTAGE = 3.3;

    // LowPassFilter
    // higher gain values -> smoother graph, more lag
    public static double GAIN = 0.95;
    LowPassFilter lowPassFilter = new LowPassFilter(GAIN);

    public MXSensor(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        /* TODO: Change channel based on corresponding wire soldered to AN (Analog Voltage) on sensor
            and plugged into REV Hub analog port pinout:
            https://docs.revrobotics.com/duo-control/control-system-overview/port-pinouts
        */
        mxSensorLeft = hwMap.get(AnalogInput.class, "MXSensorLeft");
        mxSensorRight = hwMap.get(AnalogInput.class, "MXSensorRight");
    }

    public double getDistanceMM(Side side) {
        // measured voltage -- (default to all)
        double vm = (mxSensorLeft.getVoltage() + mxSensorRight.getVoltage()) / 2.0;
        // volts per 3.3mm
        double vi = SUPPLIED_VOLTAGE / 1024;
        // range in mm
        double ri;

        switch (side) {
            case LEFT:
                vm = mxSensorLeft.getVoltage();
                break;
            case RIGHT:
                vm = mxSensorRight.getVoltage();
                break;
            case ALL:
                vm = (mxSensorLeft.getVoltage() + mxSensorRight.getVoltage()) / 2.0;
                break;
        }

        ri = 3.3 * (vm / vi);

        return ri;
    }

    public double getLowPassMM(Side side) {
        double currentValue = getDistanceMM(side);
        double estimate = lowPassFilter.estimate(currentValue);
        return estimate;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("MM", getDistanceMM(Side.ALL));
        telemetry.addData("LowPass MM", getLowPassMM(Side.ALL));
        telemetry.addData("LowPass Offset", Math.abs(getLowPassMM(Side.ALL) - getDistanceMM(Side.ALL)));

        telemetry.addData("Left MM", getDistanceMM(Side.LEFT));
        telemetry.addData("LowPass Left MM", getLowPassMM(Side.LEFT));

        telemetry.addData("Right MM", getDistanceMM(Side.RIGHT));
        telemetry.addData("LowPass Right MM", getLowPassMM(Side.RIGHT));

        telemetry.addData("left voltage", mxSensorLeft.getVoltage());
        telemetry.addData("right voltage", mxSensorRight.getVoltage());
    }

}
