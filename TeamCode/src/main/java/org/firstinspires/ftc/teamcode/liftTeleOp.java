package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;


@TeleOp(name="Basic: LiftOpMode", group="Linear Opmode")
public class liftTeleOp extends OpMode {
    // Declare OpMode members.
    private DcMotor arm = null;
    private DcMotor lift = null;
    double ticks = 751.8;
    double newTarget;
    boolean precision;

    @Override
    public void init() {
        //Hardware map
        arm = hardwareMap.get(DcMotor.class, "arm");
        lift = hardwareMap.get(DcMotor.class, "lift");

        //Telemetry
        telemetry.addData("Initalization", "complete");
        telemetry.update();

        //Encoder setup
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {


        if (gamepad1.a) {
            lift.setPower(0.5);
        }
        telemetry.addData("Motor Ticks: ", lift.getCurrentPosition());

        if (gamepad1.b) {
            reset();
        }
        lift.setPower(0);
    }
    public void liftEncode(int turnage) {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticks*turnage;
        lift.setTargetPosition((int)newTarget);
        lift.setPower(0.5); // Tweak value to increase/decrease speed
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void reset() { //resets the motors to its origin
        lift.setTargetPosition(0);
        lift.setPower(0.5);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(0);
        arm.setPower(0.5);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}