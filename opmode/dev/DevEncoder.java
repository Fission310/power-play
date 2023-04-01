package org.firstinspires.ftc.teamcode.opmode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.drivebase.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp (name = "Dev Encoder", group = "dev")
@Disabled
public class DevEncoder extends LinearOpMode {

    private Encoder parallelEncoder, perpendicularEncoder;

    Drivetrain dt = new Drivetrain(this);

    @Override
    public void runOpMode() throws InterruptedException {
        dt.init(hardwareMap);
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelOdo"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpOdo"));

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            dt.loop(gamepad1);

            telemetry.addData("parallel pos", parallelEncoder.getCurrentPosition());
            telemetry.addData("perp pos", perpendicularEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
