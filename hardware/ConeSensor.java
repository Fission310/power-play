package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ConeSensor extends Mechanism {

    private ColorSensor coneSensor;

    public static double THRESHOLD = 150;

    public ConeSensor(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        coneSensor = hwMap.get(ColorSensor.class, "coneSensor");
    }

    private int getBlue(ColorSensor color) {
        return color.blue();
    }

    private int getRed(ColorSensor color) {
        return color.red();
    }

    private boolean hasConeColor(ColorSensor color) {
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
