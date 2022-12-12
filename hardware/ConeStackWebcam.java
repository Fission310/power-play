package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ConeStackWebcam extends Mechanism {
    private WebcamName webcamName;
    private OpenCvCamera camera;
    private LineDetector detector;

    public ConeStackWebcam(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());

        webcamName = hwMap.get(WebcamName.class, "ConeStackWebcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(864, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 0);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        camera.showFpsMeterOnViewport(true);

        detector = new LineDetector(opMode.telemetry);
        camera.setPipeline(detector);
    }

    public void stopStreaming() {
        camera.stopStreaming();
    }

    private static class LineDetector extends OpenCvPipeline {
        Telemetry telemetry;
        Mat mat = new Mat();

//        static final int SCREEN_WIDTH = 864;
//        static final int SCREEN_HEIGHT = 480;
//
//        static final Rect ROI = new Rect(
//                new Point(0,0),
//                new Point(SCREEN_WIDTH, SCREEN_HEIGHT)
//        );

        public LineDetector(Telemetry t) {
            telemetry = new MultipleTelemetry(t, FtcDashboard.getInstance().getTelemetry());
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
//
            Scalar lowRedLow = new Scalar(0, 50, 50);
            Scalar highRedLow = new Scalar(10, 255, 255);
            Mat redMaskLow = new Mat();
            Core.inRange(mat, lowRedLow, highRedLow, redMaskLow);


            Scalar lowRedHigh = new Scalar(160, 50, 50);
            Scalar highRedHigh = new Scalar(180, 255, 255);
            Mat redMaskHigh = new Mat();
            Core.inRange(mat, lowRedHigh, highRedHigh, redMaskHigh);

            Mat redMask = new Mat();
            Core.bitwise_or(redMaskLow, redMaskHigh, redMask);

            List<MatOfPoint> redContours = new ArrayList<>();
            Mat redHierarchy = new Mat();
            Imgproc.findContours(redMask, redContours, redHierarchy, 1, Imgproc.CHAIN_APPROX_NONE);
            Imgproc.drawContours(input, redContours, -1, new Scalar(0, 255, 0), 2);

            int maxIndex = 0;
            double maxArea = 0;
            for (int i = 0; i < redContours.size(); i++) {
                double contourArea = Imgproc.contourArea(redContours.get(i));
                if (contourArea > maxArea) {
                    maxIndex = i;
                    maxArea = contourArea;
                }
            }

            Moments redMoments = Imgproc.moments(redContours.get(maxIndex));
            double cx = 0;
            double cy = 0;
            if (redMoments.m00 != 0) {
                cx = redMoments.m10 / redMoments.m00;
                cy = redMoments.m01 / redMoments.m00;
            }

            Imgproc.circle(input, new Point(cx, cy), 5, new Scalar(255,255,255), -1);

            telemetry.addData("cx", cx);
            telemetry.addData("cy", cy);


            redMaskLow.release();
            redMaskHigh.release();
            mat.release();
            redMask.release();
            return input;
        }
    }
}
