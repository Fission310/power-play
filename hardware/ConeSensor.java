package org.firstinspires.ftc.teamcode.hardware;

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

    private DistanceSensor coneSensor;

    public ConeSensor(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        coneSensor = hwMap.get(DistanceSensor.class, "coneSensor");
    }

    public double getDistanceMM() {
        return coneSensor.getDistance(DistanceUnit.MM);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("distance", coneSensor.getDistance(DistanceUnit.MM));
    }

}
