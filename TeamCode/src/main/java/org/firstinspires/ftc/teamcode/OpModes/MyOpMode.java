//package org.firstinspires.ftc.teamcode.OpModes;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//import org.firstinspires.ftc.teamcode.HardwareMap.Hardware;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//@Autonomous(name = "meet2Auto")
//public class MyOpMode extends LinearOpMode {
//
//    Hardware robot = new HardwareMap();
//    double armNewTarget = 0;
//    double liftNewTarget = 0;
//    private ColorSensor color1  = null;
//    private ColorSensor color2 = null;
//    public DcMotor rightFront = null;
//    public DcMotor leftFront = null;
//    public DcMotor rightBack = null;
//    public DcMotor leftBack = null;
//    public DcMotor arm, lift;
//    public Servo rightClaw = null;
//    public Servo launcher = null;
//    public Servo leftClaw = null;
//    public Servo wristLeft = null;
//    public Servo wristRight = null;
//    double ticks = 751.8;
//
//    @Override
//    public void runOpMode() {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Pose2d startPosition = new Pose2d(-36, 54, Math.toRadians(180));
//        drive.setPoseEstimate(startPosition);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
//        leftFront = hardwareMap.get(DcMotor.class, "LF");
//        rightFront = hardwareMap.get(DcMotor.class, "RF");
//        leftBack = hardwareMap.get(DcMotor.class, "LB");
//        rightBack = hardwareMap.get(DcMotor.class, "RB");
//
//        arm = hardwareMap.get(DcMotor.class, "arm");
//        lift = hardwareMap.get(DcMotor.class, "lift");
//
//        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
//        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
//        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
//        wristRight = hardwareMap.get(Servo.class, "wristRight");
//
//        launcher = hardwareMap.get(Servo.class, "planeLauncher");
//
//        rightBack.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.setDirection(DcMotor.Direction.REVERSE);
//
//        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPosition)
//
//                .addDisplacementMarker(() -> {
//                    //wristDown();
//                })
//                .forward(24)
//                .addDisplacementMarker(() -> {
//                    leftClaw.setPosition(0.5);
//                })
//
//                .lineToSplineHeading(new Pose2d(48, -34, Math.toRadians(180)))
//
//                .addDisplacementMarker(() -> {
//                    clawClose();
//                    //wristUp();
//                    armEncode(0.375, false);
//                    liftEncode(0.25, false);
//                    armEncode(0.375, false);
//                    clawOpen();
//                    liftEncode(0.25, true);
//                    armEncode(-0.75, false);
//                })
//
//                .forward(108)
//                .back(6)
//                .addDisplacementMarker(() -> {
//                    clawOpen();
//                    //wristDown();
//                    clawClose();
//                    //wristUp();
//                })
//                .back(102)
//                .addDisplacementMarker(() -> {
//                    clawClose();
//                    //wristUp();
//                    armEncode(0.375, false);
//                    liftEncode(0.45, false);
//                    armEncode(0.375, false);
//                    clawOpen();
//                    liftEncode(0.25, true);
//                    armEncode(-0.75, false);
//                })
//
//
//                .forward(102)
//                .addDisplacementMarker(() -> {
//                    clawOpen();
//                    //wristDown();
//                    clawClose();
//                    //wristUp();
//                })
//                .back(102)
//                .addDisplacementMarker(() -> {
//                    clawClose();
//                    //wristUp();
//                    armEncode(0.375, false);
//                    liftEncode(0.65, false);
//                    armEncode(0.375, false);
//                    clawOpen();
//                    liftEncode(0.25, true);
//                    armEncode(-0.75, false);
//                })
//
//                .forward(102)
//                .waitSeconds(1)
//                .back(102)
//                .waitSeconds(1)
//
//                .forward(102)
//                .waitSeconds(1)
//                .back(102)
//                .waitSeconds(1)
//
//                .build();
//
//
//        waitForStart();
//
//        if(isStopRequested()) return;
//
//        drive.followTrajectorySequence(trajSeq);
//    }
//
//    public void wristDown() {
//        wristLeft.setPosition(-gamepad2.left_stick_y);
//    }
//
//    public void wristUp() {
//        wristLeft.setPosition(0.2);
//
//    }
//
//    public void liftEncode(double turnage, boolean reset) {
//        if (reset) {
//            lift.setTargetPosition(0);
//            lift.setPower(0.2);
//            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        } else {
//            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            liftNewTarget = ticks * turnage;
//            lift.setTargetPosition((int) liftNewTarget);
//            lift.setPower(-.7);
//            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//    }
//    public void armEncode(double turnage, boolean reset) {
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armNewTarget = ticks / turnage;
//        arm.setTargetPosition((int) armNewTarget);
//        arm.setPower(.2);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    private void clawClose() {
//        leftClaw.setPosition(0.2);
//        rightClaw.setPosition(0.55);
//    }
//
//    private void clawOpen() {
//        leftClaw.setPosition(0.65);
//        rightClaw.setPosition(0.1);
//    }
//
//
//
//}
