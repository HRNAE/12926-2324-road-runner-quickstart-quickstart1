//package org.firstinspires.ftc.teamcode.OpModes;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.teamcode.HardwareMap.Hware;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//import java.util.List;
//
//@Autonomous(name = "meet1Auto")
// public class meet1Auto extends LinearOpMode {
//    Hware robot = new Hware();
//    ColorSensor leftSensor = null;
//    ColorSensor rightSensor = null;
//    int leftD, rightD = 0;
////    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
////
////    /**
////     * {@link #tfod} is the variable to store our instance of the TensorFlow Object Detection processor.
////     */
////    private TfodProcessor tfod;
////
////    /**
////     * {@link #visionPortal} is the variable to store our instance of the vision portal.
////     */
////    private VisionPortal visionPortal;
////    int location = 0;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
////        telemetryTfod();
//        telemetry.update();
//
//        waitForStart();
//        //if conditions
//        drive(1000, 1000, 0.3);
//
//    }
//
//    private void drive(double left, double right, double speed) {
//        leftD += left;
//        rightD += right;
//
//        //leftFront.setTargetPosition(left);
//        robot.leftBack.setTargetPosition((int) left);
//        //rightFront.setTargetPosition(right);
//        robot.rightBack.setTargetPosition((int) right);
//
//        //leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        //leftFront.setPower(speed - 0.05);
//        robot.leftBack.setPower(speed - 0.05);
//        //rightFront.setPower(speed);
//        robot.rightBack.setPower(speed);
//
//        robot.leftClaw.setPosition(0);
//        robot.rightClaw.setPosition(0.695);
//        if (getRuntime() > 7) {
//            robot.wristRight.setPosition(0.45);
//            robot.wristLeft.setPosition(0.45);
//        }
//        if (getRuntime() > 8) {
//            robot.leftClaw.setPosition(0.5);
//            robot.rightClaw.setPosition(0.3);
//        }
//
//        while (opModeIsActive()) { //If the motors are running, then it wont move other motors
//        }
//    }
//    private void initTfod() {
//
//        // Create the TensorFlow processor the easy way.
//        tfod = TfodProcessor.easyCreateWithDefaults();
//        // Indicate that only the zoomed center area of each                //NEW CODE
//        // image will be passed to the TensorFlow object                    //^
//        // detector. For no zooming, set magnification to 1.0.              //^
//        tfod.setZoom(1.5);                                                  //^
//        tfod.setMinResultConfidence((float) 0.9);                          //NEW CODE(CONFIDENCE MINIMUM)
//        // Create the vision portal the easy way.
//        if (USE_WEBCAM) {
//            visionPortal = VisionPortal.easyCreateWithDefaults(
//                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
//        } else {
//            visionPortal = VisionPortal.easyCreateWithDefaults(
//                    BuiltinCameraDirection.BACK, tfod);
//        }
//
//    }   // end method initTfod()
//
//    /**
//     * Function to add telemetry about TensorFlow Object Detection (TFOD) recognitions.
//     */
//    private void telemetryTfod() {
//
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        telemetry.addData("# Objects Detected", currentRecognitions.size());
//
//        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//
//            telemetry.addData(""," ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//        }   // end for() loop
//
//    }   // end method telemetryTfod()
//} // end class
//
