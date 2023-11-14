 package org.firstinspires.ftc.teamcode;
        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.geometry.Vector2d;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorController;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.ColorSensor;


        import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
        import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "meet1Auto")
public class redRight extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    DcMotor lift = null;
    DcMotor arm = null;
    ColorSensor leftSensor = null;
    ColorSensor rightSensor = null;
    private Servo rightClaw = null;
    private Servo leftClaw = null;
    private Servo launcher = null;
    private Servo wrist = null;
    double movement;
    double rotation = 1405.0;
    double strafe;
    double ticks = 751.8;
    double newTarget;
    double armNewTarget;
    double index = -0.33;
    int leftD, rightD = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        leftSensor = hardwareMap.get(ColorSensor.class, "leftSensor");
        rightSensor = hardwareMap.get(ColorSensor.class, "rightSensor");
        arm = hardwareMap.get(DcMotor.class, "arm");
        lift = hardwareMap.get(DcMotor.class, "lift");

        leftFront = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        leftBack = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");


        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        launcher = hardwareMap.get(Servo.class, "planeLauncher");
        wrist = hardwareMap.get(Servo.class, "wrist");

        //leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftD = 0;
        rightD = 0;

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        drive(-1000, -1000, 0.3);
        drive(5000, 5000, 0.3);
    }

    private void drive(double left, double right, double speed) {
        leftD += left;
        rightD += right;

        //leftFront.setTargetPosition(left);
        leftBack.setTargetPosition((int)left);
        //rightFront.setTargetPosition(right);
        rightBack.setTargetPosition((int)right);

        //leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //leftFront.setPower(speed - 0.05);
        leftBack.setPower(speed - 0.05);
        //rightFront.setPower(speed);
        rightBack.setPower(speed);

        leftClaw.setPosition(0);
        rightClaw.setPosition(0.695);
        if (getRuntime() > 7) {
            wrist.setPosition(0.45);
        }
        if (getRuntime() > 8) {
            leftClaw.setPosition(0.5);
            rightClaw.setPosition(0.3);
        }




        while (opModeIsActive()) { //If the motors are running, then it wont move other motors
            /*if (false) {
                idle();
            } else {

                if (getRuntime() > 5) {
                    int x = leftSensor.red(); //left
                    int y = leftSensor.blue();
                    int z = leftSensor.green();
                    int x2 = rightSensor.red(); //right
                    int y2 = rightSensor.blue();
                    int z2 = rightSensor.green();

                    if (x > 27 || z > 25) { //left
                        drive(0, 3100, 0.4);
                        idle();
                       // if (getRuntime() > 7) {
                            wrist.setPosition(0.45);
                            leftClaw.setPosition(0.5);
                            rightClaw.setPosition(0.3);
                        }
                    } else if (x2 < 35 || z2 > 30) { //right

                       /*
                        if (getRuntime() > 7) {
                            wrist.setPosition(0.45);
                            leftClaw.setPosition(0.5);
                            rightClaw.setPosition(0.3);
                        }
                    } else if ((getRuntime() > 8)) { // center
                            //drive(1150 ,1150, 0.4);

                            wrist.setPosition(0.45);
                            leftClaw.setPosition(0.5);
                            rightClaw.setPosition(0.3);
                    } else {
                        leftClaw.setPosition(0.5);
                        rightClaw.setPosition(0.3);
                    }

                }

            }
            */
        }

    }
}

