package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class ConeSensor extends Mechanism {

    private ColorSensor coneSensor;

    public static double THRESHOLD = 150;

    public ConeSensor(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        coneSensor = hwMap.get(ColorSensor.class, "coneSensor");
    }

    public boolean hasCone() {
        if (( coneSensor.red() >= THRESHOLD ) || ( coneSensor.blue() >= THRESHOLD )) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("hasCone", hasCone());
    }

}
