package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "RED 69")

public class test2AutoRed extends LinearOpMode {
    double LBnewTarget, LFnewTarget, RBnewTarget, RFnewTarget = 0;
    double ticks = 537.7;
    Hware robot;
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution(Old: 640)
    private static final int CAMERA_HEIGHT = 960; // height of wanted camera resolution(Old : 360)

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels


    @Override
    public void runOpMode() {
        robot = new Hware(hardwareMap);
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.update();
            if (getRuntime() > 3) {
                if (cX < 320) {
                    telemetry.addData("Direction: ", "left");
                } else if (cX > 960) {
                    telemetry.addData("Direction: ", "right");
                } else if (cX > 320 && cX < 960) {
                    telemetry.addData("Direction: ", "center");
                }
            }
            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(100, 100, 100);
            Scalar upperYellow = new Scalar(200, 255, 255);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
    public void encoderLF(double turnage)
    {
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFnewTarget = ticks * turnage;
        robot.leftFront.setTargetPosition((int)LFnewTarget);
        robot.leftFront.setPower(1);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFnewTarget = 0;
    }
    public void encoderRF(double turnage)
    {
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFnewTarget = ticks * turnage;
        robot.rightFront.setTargetPosition((int)RFnewTarget);
        robot.rightFront.setPower(1);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFnewTarget = 0;
    }
    public void encoderLB(double turnage)
    {
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBnewTarget = ticks * turnage;
        robot.leftBack.setTargetPosition((int)LBnewTarget);
        robot.leftBack.setPower(1);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBnewTarget = 0;  //NEW CHANGE
    }
    public void encoderRB(double turnage)
    {
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBnewTarget = ticks * turnage;
        robot.rightBack.setTargetPosition((int)RBnewTarget);
        robot.rightBack.setPower(1);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBnewTarget = 0;
    }
    public void strafe(double magnitude) {
        encoderLB(-magnitude);
        encoderLF(magnitude);
        encoderRB(magnitude);
        encoderRF(-magnitude);
        sleep(1000);
    }

    public void negSlopeDiagonalStrafe(double magnitude) {
        encoderLB(-magnitude);
        encoderLF(0);
        encoderRB(0);
        encoderRF(-magnitude);
        sleep(1000);
    }
    public void posSlopeDiagonalStrafe(double magnitude) {
        encoderLB(0);
        encoderLF(-magnitude);
        encoderRB(-magnitude);
        encoderRF(0);
        sleep(1000);
    }
    public void forward(double magnitude) {
        encoderLB(magnitude);
        encoderLF(magnitude);
        encoderRB(magnitude);
        encoderRF(magnitude);
        sleep(1000);
    }

    private void turn(double magnitude) {
        encoderLB(-magnitude / 50);
        encoderLF(-magnitude / 50);
        encoderRB(magnitude / 50);
        encoderRF(magnitude / 50);
    }
    private void clawClose() {
        robot.leftClaw.setPosition(0.2);
        robot.rightClaw.setPosition(0.55);
    }

    private void clawOpen() {
        robot.leftClaw.setPosition(0.65);
        robot.rightClaw.setPosition(0.1);
    }
    public void wristDown() { robot.wristRight.setPosition(1);}

    public void wristUp() {
        robot.wristRight.setPosition(0);
    }
}