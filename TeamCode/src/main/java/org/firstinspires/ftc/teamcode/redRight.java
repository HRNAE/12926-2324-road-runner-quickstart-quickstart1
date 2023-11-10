package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;




import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;




@Autonomous(name = "redRight", group = "drive")
public class redRight extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    DcMotor Lift1 = null;
    DcMotor Arm1 = null;
    ColorSensor leftSensor = null;
    ColorSensor rightSensor = null;
    private Servo rightClaw = null;
    private Servo leftClaw = null;
    private Servo launcher = null;
    private Servo wrist = null;
    double movement;
    double rotation;
    double strafe;
    double ticks = 751.8;
    double newTarget;
    double  armNewTarget;
    double index = -0.33;


    int x, y, z, y2, x2, z2;




   /*
   private void liftControl(double power) {
       Lift1.setPower(power);
       Arm1.setPower(power);
   }


    */


    @Override
    public void runOpMode() {


        leftSensor = hardwareMap.get(ColorSensor.class,"leftSensor");
        rightSensor = hardwareMap.get(ColorSensor.class, "rightSensor");
        Arm1 = hardwareMap.get(DcMotor.class, "arm");
        Lift1 = hardwareMap.get(DcMotor.class, "lift");
        leftFront  = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        leftBack = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        launcher = hardwareMap.get(Servo.class, "planeLauncher");
        wrist = hardwareMap.get(Servo.class, "wrist");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose2 = new Pose2d(11, -36, 90);
        Pose2d startPose1 = new Pose2d(11, -72, 90);


        //drive.setPoseEstimate(startPose);
        TrajectorySequence forward = drive.trajectorySequenceBuilder(startPose2)
                .forward(36)
                .build();
        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose2)
                .strafeLeft(11)
                /*       .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                           leftClaw.setPosition(0.73);
                       })
                       .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                           wrist.setPosition(0.73); //value will be changed
                       })


                 */
                .waitSeconds(2)
                .strafeRight(11)
                // .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                //        wrist.setPosition(0.445);
                //   })
                .turn(Math.toRadians(-90))
                .forward(33)
                .strafeLeft(5.5)
                /*
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    double degree = -5.0; //will change later
                    Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armNewTarget = ticks / degree;
                    Arm1.setTargetPosition((int) armNewTarget);
                    Arm1.setPower(-.2);
                    Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    degree = -1.0;
                    Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    newTarget = ticks / degree;
                    Lift1.setTargetPosition((int) newTarget);
                    Lift1.setPower(-.2);
                    Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setPosition(0.73);
                })


                 */
                .waitSeconds(3)
                /*
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    rightClaw.setPosition(0.73); // will change later
                })
                .waitSeconds(3)
                .turn(Math.toRadians(-180))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Lift1.setTargetPosition(0);
                    Lift1.setPower(0.2);
                    Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm1.setTargetPosition(0);
                    Arm1.setPower(0.2);
                    Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightClaw.setPosition(-0.33);
                    leftClaw.setPosition(1);
                    //launcher.setPosition(0.5);
                    wrist.setPosition(0.93);
                })


                 */
                .build();


        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose2)
                .strafeRight(11)
                /*
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    leftClaw.setPosition(0.73);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    wrist.setPosition(0.73); //value will be changed
                })


                 */
                .waitSeconds(2)
                .strafeRight(11)
                /*
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    wrist.setPosition(0.445);
                })


                 */
                .turn(Math.toRadians(-90))
                .forward(33)
                .strafeRight(5.5)
                /*
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    double degree = -5.0; //will change later
                    Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armNewTarget = ticks / degree;
                    Arm1.setTargetPosition((int) armNewTarget);
                    Arm1.setPower(-.2);
                    Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    degree = -1.0;
                    Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    newTarget = ticks / degree;
                    Lift1.setTargetPosition((int) newTarget);
                    Lift1.setPower(-.2);
                    Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setPosition(0.73);
                })


                 */
                .waitSeconds(3)
                /*
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    rightClaw.setPosition(0.73); // will change later
                })
                .waitSeconds(3)
                .turn(Math.toRadians(-180))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Lift1.setTargetPosition(0);
                    Lift1.setPower(0.2);
                    Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm1.setTargetPosition(0);
                    Arm1.setPower(0.2);
                    Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightClaw.setPosition(-0.33);
                    leftClaw.setPosition(1);
                    //launcher.setPosition(0.5);
                    wrist.setPosition(0.93);
                })


                 */
                .build();


        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose2)
                .forward(2.5)
                /*
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    leftClaw.setPosition(0.73);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    wrist.setPosition(0.73); //value will be changed
                })


                 */
                .waitSeconds(2)
                .back(2.5)
                /*
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    wrist.setPosition(0.445);
                })


                 */
                .turn(Math.toRadians(-90))
                .forward(33)
                /*
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    double degree = -5.0; //will change later
                    Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armNewTarget = ticks / degree;
                    Arm1.setTargetPosition((int) armNewTarget);
                    Arm1.setPower(-.2);
                    Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    degree = -1.0;
                    Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    newTarget = ticks / degree;
                    Lift1.setTargetPosition((int) newTarget);
                    Lift1.setPower(-.2);
                    Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setPosition(0.73);
                })
                /*
                 */
                .waitSeconds(3)
                /*
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    rightClaw.setPosition(0.73); // will change later
                })
                .waitSeconds(3)
                .turn(Math.toRadians(-180))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Lift1.setTargetPosition(0);
                    Lift1.setPower(0.2);
                    Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm1.setTargetPosition(0);
                    Arm1.setPower(0.2);
                    Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightClaw.setPosition(-0.33);
                    leftClaw.setPosition(1);
                    //launcher.setPosition(0.5);
                    wrist.setPosition(0.93);
                })


                 */
                .build();
        waitForStart();




        //telemetry.addData("Type", detectedColor);
        //telemetry.update();


        while (opModeIsActive()) {
            int x = leftSensor.red(); //left
            int y = leftSensor.blue();
            int z = leftSensor.green();
            int x2 = rightSensor.red(); //right
            int y2 = rightSensor.blue();
            int z2 = rightSensor.green();
            telemetry.addData("", x);
            telemetry.addData("", y);
            telemetry.addData("", z);
            telemetry.addData("", x2);
            telemetry.addData("", y2);
            telemetry.addData("", z2);
            telemetry.update();
            drive.followTrajectorySequence(forward);
            if (x > 25 && x > y && x > z) { //left
                drive.followTrajectorySequence(left);
            } else if (x2 > 25 && x2 > y2 && x2 > z2) { //right
                drive.followTrajectorySequence(right);
            } else { // center
                drive.followTrajectorySequence(center);
            }
        }
    };
}

