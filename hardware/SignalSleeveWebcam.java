package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class SignalSleeveWebcam extends Mechanism {

    public enum ROBOT_SIDE {
        CONTROL_HUB,
        EXPANSION_HUB
    }
    private ROBOT_SIDE robot_side;


    private WebcamName webcamName;
    private OpenCvCamera camera;
    private SideDetector detector;
    private String deviceName;

    private static final int WIDTH = 864;
    private static final int HEIGHT = 480;

    public SignalSleeveWebcam(LinearOpMode opMode, String deviceName, ROBOT_SIDE robot_side) {
        this.opMode = opMode;
        this.deviceName = deviceName;
        this.robot_side = robot_side;
    }

    public void init(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());

        webcamName = hwMap.get(WebcamName.class, deviceName);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.SIDEWAYS_RIGHT); // 864, 480
                FtcDashboard.getInstance().startCameraStream(camera, 0);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        camera.showFpsMeterOnViewport(true);

        detector = new SideDetector(opMode.telemetry, deviceName, robot_side);
        camera.setPipeline(detector);
    }

    public enum Side {
        ONE,
        TWO,
        THREE,
        NOT_FOUND
    }

    public Side side() {
        return detector.getSide();
    }

    public void stopStreaming() {
        camera.stopStreaming();
    }

    private static class SideDetector extends OpenCvPipeline {

        Telemetry telemetry;
        Mat mat = new Mat();

        private Side side;

        static final int SCREEN_WIDTH = HEIGHT;
        static final int SCREEN_HEIGHT = WIDTH;

        static final int FIRST_THIRD_WIDTH = SCREEN_WIDTH/3 + 10;
        static final int SECOND_THIRD_WIDTH = FIRST_THIRD_WIDTH * 2 - 10;

        static final int FIRST_THIRD_HEIGHT = SCREEN_HEIGHT/3;
        static final int SECOND_THIRD_HEIGHT = FIRST_THIRD_HEIGHT*2 - 30;

        static final Rect RIGHT_ROI = new Rect(
                new Point(FIRST_THIRD_WIDTH + 100, FIRST_THIRD_HEIGHT),
                new Point(SECOND_THIRD_WIDTH + 100, SECOND_THIRD_HEIGHT)
        );

        static final Rect LEFT_ROI = new Rect(
                new Point(FIRST_THIRD_WIDTH - 100, FIRST_THIRD_HEIGHT),
                new Point(SECOND_THIRD_WIDTH - 100, SECOND_THIRD_HEIGHT)
        );

        private String deviceName;
        private ROBOT_SIDE robot_side;

        public SideDetector(Telemetry t, String deviceName, ROBOT_SIDE robot_side) {
            telemetry = new MultipleTelemetry(t, FtcDashboard.getInstance().getTelemetry());
            this.deviceName = deviceName;
            this.robot_side = robot_side;
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

            Scalar lowMagenta = new Scalar(155, 50, 50);
            Scalar highMagenta = new Scalar(165, 255, 255);
            Mat magentaMask = new Mat();
            Core.inRange(mat, lowMagenta, highMagenta, magentaMask);

            Scalar lowGreen = new Scalar(40, 50, 50);
            Scalar highGreen = new Scalar(75, 255, 255);
            Mat greenMask = new Mat();
            Core.inRange(mat, lowGreen, highGreen, greenMask);

            Scalar lowYellow = new Scalar(23, 50, 70);
            Scalar highYellow = new Scalar(32, 255, 255);

            Mat yellowMask = new Mat();
            Core.inRange(mat, lowYellow, highYellow, yellowMask);

            Rect ROI;
            if (robot_side == ROBOT_SIDE.CONTROL_HUB) {
                ROI = RIGHT_ROI;
            } else {
                ROI = LEFT_ROI;
            }


            double magentaValue, greenValue, yellowValue;

            magentaValue = Core.sumElems(magentaMask).val[0] / ROI.area() / 255;
            greenValue = Core.sumElems(greenMask).val[0] / ROI.area() / 255;
            yellowValue = Core.sumElems(yellowMask).val[0] / ROI.area() / 255;

            telemetry.addData("magentaVal", magentaValue);
            telemetry.addData("greenVal", greenValue);
            telemetry.addData("yellowVal", yellowValue);

            magentaMask.release();
            greenMask.release();
            yellowMask.release();

            double maxVal = Math.max(Math.max(magentaValue, greenValue), yellowValue);

            telemetry.addData("maxVal", maxVal);

            boolean isMagenta = maxVal == magentaValue;
            boolean isGreen = maxVal == greenValue;
            boolean isYellow = maxVal == yellowValue;

            if (isMagenta) {
                side = Side.ONE;
                telemetry.addData("Side", "1 | magenta");
            } else if (isGreen) {
                side = Side.TWO;
                telemetry.addData("Side", "2 | green");
            } else if (isYellow) {
                side = Side.THREE;
                telemetry.addData("Side", "3 | yellow");
            } else {
                side = Side.NOT_FOUND;
                telemetry.addData("Side", "not found");
            }
            telemetry.update();

            Scalar colorOne = new Scalar(255, 0, 255);
            Scalar colorTwo = new Scalar(0, 255, 0);
            Scalar colorThree = new Scalar(255, 255, 0);
            Scalar colorNotFound = new Scalar(255, 0, 0);

            if (side == Side.ONE) {
                Imgproc.rectangle(input, ROI, colorOne, 5);
            } else if (side == Side.TWO) {
                Imgproc.rectangle(input, ROI, colorTwo, 5);
            } else if (side == Side.THREE) {
                Imgproc.rectangle(input, ROI, colorThree, 5);
            } else {
                Imgproc.rectangle(input, ROI, colorNotFound, 10);
            }

            mat.release();
            return input;
        }

        public Side getSide() {
            return side;
        }
    }
}
