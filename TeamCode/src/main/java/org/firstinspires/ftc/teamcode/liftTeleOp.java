/*package org.firstinspires.ftc.teamcode;

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
    public double ticks = 751.8;
   public double newTarget;
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

    }

    @Override
    public void loop() {


        if (gamepad1.a) {

        }
        telemetry.addData("Motor Ticks: ", lift.getCurrentPosition());

        if (gamepad1.b) {
            reset();
        }

    }

    public void reset() { //resets the motors to its origin
        lift.setTargetPosition(0);
        lift.setPower(0.5);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}


 */