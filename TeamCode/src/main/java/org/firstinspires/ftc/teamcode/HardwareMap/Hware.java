package org.firstinspires.ftc.teamcode.HardwareMap;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hware {
    public DcMotor rightFront = null;
    public DcMotor leftFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftBack = null;
    public DcMotor arm, lift;
    public Servo rightClaw, leftClaw, launcher, wrist;

    public double ticks = 751.8;
    public double liftNewTarget;
    public double armNewTarget;

    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();


    public void initialize(HardwareMap hwMap) {
        hardwareMap = hwMap;

        //initialize Chassis
        leftFront = hwMap.get(DcMotor.class, "LF");
        rightFront = hwMap.get(DcMotor.class, "RF");
        leftBack = hwMap.get(DcMotor.class, "LB");
        rightBack = hwMap.get(DcMotor.class, "RB");

        //Set motor directions
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        //initialize slide motors
        arm = hwMap.get(DcMotor.class, "arm");
        lift = hwMap.get(DcMotor.class, "lift");

        //Set motor modes
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialize Servos
        rightClaw = hwMap.get(Servo.class, "rightClaw");
        leftClaw = hwMap.get(Servo.class, "leftClaw");
        launcher = hwMap.get(Servo.class, "planeLauncher");
        wrist = hwMap.get(Servo.class, "wrist");

        //Set Zero Power Behavior
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set Power
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void clawClose() {
        leftClaw.setPosition(0);
        rightClaw.setPosition(0.695);
    }

    public void clawOpen() {
        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0.3);
    }

    public void wristDown() {
        wrist.setPosition(0.5);
    }

    public void wristUp() {
        wrist.setPosition(0.85);
    }

    public void liftEncode(double turnage, boolean reset) {
        if (reset) {
            lift.setTargetPosition(0);
            lift.setPower(0.2);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftNewTarget = ticks * turnage;
            lift.setTargetPosition((int) liftNewTarget);
            lift.setPower(-.4);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void armEncode(double turnage, boolean reset) {
        if (reset) {
            arm.setTargetPosition(0);
            arm.setPower(0.2);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armNewTarget = ticks * turnage;
            arm.setTargetPosition((int) armNewTarget);
            arm.setPower(-.1);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


    }


}