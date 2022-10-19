package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Class for Ultrasonic Analog Sensors
public class UASensor extends Mechanism {

    /* TODO: sensor pcp -> hub
        black - ground -> ground
        red - V+ (next to ground) -> 3.3V
        white - nothing
        blue - 3 -> 0
    */

    // Analog Sensors, they report voltage
    private AnalogInput uaSensor;
    private AnalogInputController uaSensorController;
    private int channel = -1;

    // rev hubs supply 3.3 volts to analog ports
    double SUPPLIED_VOLTAGE = 3.3;

    public UASensor(LinearOpMode opMode, int channel) {
        this.opMode = opMode;
        this.channel = channel;
    }

    @Override
    public void init(HardwareMap hwMap) {
        uaSensorController = hwMap.get(AnalogInputController.class, "UASensor");

        /* TODO: Change channel based on corresponding wire soldered to AN (Analog Voltage) on sensor
            and plugged into REV Hub analog port pinout:
            https://docs.revrobotics.com/duo-control/control-system-overview/port-pinouts
        */
        uaSensor = new AnalogInput(uaSensorController, channel);
    }

    private double getDistanceMM() {

        // Analog sensor, get voltage readout from specified channel above
        // Vm = measured voltage
        double Vm = uaSensor.getVoltage();

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
