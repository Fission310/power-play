package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.MXSensor;

import java.util.List;

@TeleOp (name = "Dev MXSensor", group = "dev")
public class DevMXSensor extends LinearOpMode {

    private MXSensor mxSensor = new MXSensor(this);

    @Override
    public void runOpMode() throws InterruptedException {
        mxSensor.init(hardwareMap);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Bulk Reading
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // clear cache
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            mxSensor.telemetry(telemetry);
            telemetry.update();
        }
    }
}
