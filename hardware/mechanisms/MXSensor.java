package org.firstinspires.ftc.teamcode.hardware.mechanisms;

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

    /* TODO: sensor pcp -> hub
        black - ground -> ground
        red - V+ (next to ground) -> 3.3V
        white - nothing
        blue - 3 -> 0
    */

    // Analog Sensors, they report voltage
    private AnalogInput mxSensor;



    // rev hubs supply 3.3 volts to analog ports
    double SUPPLIED_VOLTAGE = 3.3;

    // LowPassFilter
    // higher gain values -> smoother graph, more lag
    public static double GAIN = 0.95;
    LowPassFilter lowPassFilter = new LowPassFilter(GAIN);

    public static double Q = 1;
    public static double R = 1;
    public static int N = 5;
    KalmanFilter kalmanFilter = new KalmanFilter(Q, R, N);

    public MXSensor(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        /* TODO: Change channel based on corresponding wire soldered to AN (Analog Voltage) on sensor
            and plugged into REV Hub analog port pinout:
            https://docs.revrobotics.com/duo-control/control-system-overview/port-pinouts
        */
        mxSensor = hwMap.get(AnalogInput.class, "MXSensorRight");
    }

    public double getDistanceMM() {
        // measured voltage -- (default to all)
        double vm = mxSensor.getVoltage();
        // volts per 3.3mm
        double vi = SUPPLIED_VOLTAGE / 1024;
        // range in mm
        double ri = 3.3 * (vm / vi);

        return ri;
    }

    public double getLowPassMM() {
        double currentValue = getDistanceMM();
        double estimate = lowPassFilter.estimate(currentValue);
        return estimate;
    }

    public double getKalmanMM() {
        double currentValue = getDistanceMM();
        double estimate = kalmanFilter.estimate(currentValue);
        return estimate;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("distance", getDistanceMM());
        telemetry.addData("lowpass", getLowPassMM());
        telemetry.addData("kalman", getKalmanMM());
    }

}
