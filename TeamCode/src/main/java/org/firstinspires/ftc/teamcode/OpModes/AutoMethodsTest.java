package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Methods Testing")

public class AutoMethodsTest extends LinearOpMode {
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

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_right) {
                strafe(1);
            }
            if (gamepad1.dpad_left) {
                strafe(-1);
            }
            if (gamepad1.dpad_up) {
                forward(1);
            }
            if (gamepad1.dpad_down) {
                forward(-1);
            }
            if (gamepad1.a) {
                posSlopeDiagonalStrafe(1.414);
            }
            if (gamepad1.y) {
                negSlopeDiagonalStrafe(-1.414);
            }
            if (gamepad1.x) {
                turn(90);
            }
            if (gamepad1.b) {
                turn(-90);
            }
        }
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