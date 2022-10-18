package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MXSensor extends Mechanism {

    /* TODO: sensor pcp -> hub
        black - ground -> ground
        red - V+ (next to ground) -> 3.3V
        white - nothing
        blue - 3 -> 0
    */

    // Analog Sensors, they report voltage
    private AnalogInput mxSensor;
    private AnalogInputController mxSensorController;
    private int channel = -1;

    // rev hubs supply 3.3 volts to analog ports
    double SUPPLIED_VOLTAGE = 3.3;

    public MXSensor(LinearOpMode opMode, int channel) {
        this.opMode = opMode;
        this.channel = channel;
    }

    @Override
    public void init(HardwareMap hwMap) {
        mxSensorController = hwMap.get(AnalogInputController.class, "MXSensor");

        /* TODO: Change based on corresponding wire soldered to AN (Analog Voltage) on sensor
            and plugged into REV Hub analog port pinout:
            https://docs.revrobotics.com/duo-control/control-system-overview/port-pinouts
        */
        mxSensor = new AnalogInput(mxSensorController, channel);
    }

    private double getDistanceMM() {

        // Analog sensor, get voltage readout from specified channel above
        // Vm = measured voltage
        double Vm = mxSensor.getVoltage();

        // convert voltage to distance

        // Vi = volts per 5mm
        double Vi = SUPPLIED_VOLTAGE / 1024;

        // Ri = range in mm
        double Ri = 5 * (Vm / Vi);

        return Ri;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("dist in mm:", getDistanceMM());
    }

}
