package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class testOpMode extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
 /*   private Servo leftServo;
    private Servo rightServo;
    private DcMotor Lift1;
    private DcMotor Lift2;
    private Servo leftRotate;
    private Servo rightRotate;
    private Servo dumpServo;
    private Servo dumpClaw;
    private DcMotor ExtLift;
//    private RevBlinkinLedDriver lights;
*/
    double movement;
    double rotation;
    double strafe;
/*
    public void liftPos (int pos) { // Receives position value and sets lift to go to that position
        Lift1.setTargetPosition(-pos);
        Lift2.setTargetPosition(pos);
        Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
*/
//    // 14524 Driver Controls
//    private void driverControl (boolean check) {
//        movement = -gamepad1.left_stick_y;
//        strafe = gamepad1.left_stick_x;
//        rotation = gamepad1.right_stick_x;
//
//        if (check) {
//            leftFront.setPower(movement + strafe + rotation);
//            rightFront.setPower(movement - strafe - rotation);
//            leftBack.setPower(movement - strafe + rotation);
//            rightBack.setPower(movement + strafe - rotation);
//        }
//        else {
//            driverControl();
//        }
//    }

    // 12926 Driver Controls
    private void driverControl() {

        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double direction = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        boolean precision = gamepad1.right_bumper;

        //INFO Increasing speed to a maximum of 1
        double lf = magnitude * Math.sin(direction + Math.PI / 4) + rotation;
        double lb = magnitude * Math.cos(direction + Math.PI / 4) + rotation;
        double rf = magnitude * Math.cos(direction + Math.PI / 4) - rotation;
        double rb = magnitude * Math.sin(direction + Math.PI / 4) - rotation;

        double hypot = Math.hypot(movement, strafe);
        double ratio;
        if (movement == 0 && strafe == 0)
            ratio = 1;
        else if(precision)
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf))) / 2;
        else
            ratio = hypot / (Math.max(Math.max(Math.max(Math.abs(lf), Math.abs(lb)), Math.abs(rb)), Math.abs(rf)));

        leftFront.setPower(ratio * lf);
        leftBack.setPower(ratio * lb);
        rightFront.setPower(ratio * rf);
        rightBack.setPower(ratio * rb);
    }
/*
    //controls lift motors
    private void liftControl (double power) {
        Lift1.setPower(power);
        Lift2.setPower(power);
    }


    private void resetLift() {
        if(gamepad1.x){
            Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    //    //Lifts the lift
    private void cascadinglift() {
//        if(Lift1.getCurrentPosition() = 0 && Lift2.getCurrentPosition() = 0.0) {
        if(gamepad2.cross) {
            liftPos(1250);
            liftControl(1);
        } else if (gamepad2.circle){
            Lift1.setTargetPosition(-2068);
            Lift2.setTargetPosition(2068);
            Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
        } else if (gamepad2.triangle){
            Lift1.setTargetPosition(-2852);
            Lift2.setTargetPosition(2852);
            Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
        } else if( gamepad2.square) {
            Lift1.setTargetPosition(0);
            Lift2.setTargetPosition(0);
            Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
        }
        else if( gamepad2.dpad_right) {
            Lift1.setTargetPosition(-420);
            Lift2.setTargetPosition(420);
            Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(1);
        }
        else if(gamepad2.dpad_up){
            Lift1.setTargetPosition(Lift1.getCurrentPosition() - 50);
            Lift2.setTargetPosition(Lift2.getCurrentPosition() + 50);
            Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(.5);
        }
        else if(gamepad2.dpad_down){
            Lift1.setTargetPosition(Lift1.getCurrentPosition() + 50);
            Lift2.setTargetPosition(Lift2.getCurrentPosition() - 50);
            Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftControl(.5);
        }
    }




    private void dumpMovement(){
         if(gamepad2.right_bumper){
              rightRotate.setPosition(0);
              leftRotate.setPosition(0);
          } else {
              rightRotate.setPosition(1);
              leftRotate.setPosition(1);
          }
          if(gamepad2.left_bumper){
              dumpServo.setPosition(.835);
          } else {
              dumpServo.setPosition(.05);
          }
          if(gamepad2.right_trigger > 0){
              dumpClaw.setPosition(0);
          } else {
              dumpClaw.setPosition(1);
          }
      }
*/

    private void test(){
        if (gamepad1.cross){
            rightFront.setPower(1);
            rightBack.setPower(0);
            leftFront.setPower(0);
            leftBack.setPower(0);
        } else if (gamepad1.square) {
            rightFront.setPower(0);
            rightBack.setPower(1);
            leftFront.setPower(0);
            leftBack.setPower(0);
        } else if (gamepad1.triangle) {
            rightFront.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(1);
            leftBack.setPower(0);
        } else if (gamepad1.circle) {
            rightFront.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            leftBack.setPower(1);
        }
    }

    private void telemetry() {
        telemetry.addData("FL Power", Double.parseDouble(JavaUtil.formatNumber(leftFront.getPower(), 2)));
        telemetry.addData("BL Power", Double.parseDouble(JavaUtil.formatNumber(leftBack.getPower(), 2)));
        telemetry.addData("FR Power", Double.parseDouble(JavaUtil.formatNumber(rightFront.getPower(), 2)));
        telemetry.addData("BR Power", Double.parseDouble(JavaUtil.formatNumber(rightBack.getPower(), 2)));
 //       telemetry.addData("Lift 1 position", Double.parseDouble(JavaUtil.formatNumber(Lift1.getCurrentPosition(), 2)));
  //      telemetry.addData("Lift 2 position", Double.parseDouble(JavaUtil.formatNumber(Lift2.getCurrentPosition(), 2)));
    //    telemetry.addData("Angle Lift Position", Double.parseDouble(JavaUtil.formatNumber(ExtLift.getCurrentPosition(), 2)));
        telemetry.update();
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        leftBack = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        boolean changeDrive = true;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if(opModeIsActive()){
            while (opModeIsActive()) {
//
//                if (gamepad1.left_bumper && !gamepad1.right_bumper) {
//                    changeDrive = true;
//                } else if (!gamepad1.left_bumper && gamepad1.right_bumper) {
//                    changeDrive = false;
//                }

                driverControl();
                //resetLift();
                //cascadinglift();
                // test();
                // dumpMovement();
                telemetry();
            }
        }
    }
}



