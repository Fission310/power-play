package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ConeSensor extends Mechanism {

    private ColorRangeSensor coneSensor;

    public static double THRESHOLD = 100;

    public ConeSensor(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        coneSensor = hwMap.get(ColorRangeSensor.class, "coneSensor");
    }

    private int getBlue(ColorRangeSensor color) {
        return color.blue();
    }

    private int getRed(ColorRangeSensor color) {
        return color.red();
    }

    private boolean hasConeColor(ColorRangeSensor color) {
        int blue = getBlue(color);
        int red = getRed(color);

        return ((blue >= THRESHOLD) || (red >= THRESHOLD));
    }

    public boolean hasCone() {
        return hasConeColor(coneSensor);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("has cone?", hasCone());
        telemetry.addData("blue value", getBlue(coneSensor));
        telemetry.addData("red value", getRed(coneSensor));
    }

}
