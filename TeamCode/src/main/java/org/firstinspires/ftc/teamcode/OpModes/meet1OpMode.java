
    package org.firstinspires.ftc.teamcode.OpModes;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.ColorSensor;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;

    import org.firstinspires.ftc.teamcode.HardwareMap.Hware;

    @TeleOp(name="Meet1OpMode", group="Linear Opmode")
public class meet1OpMode extends LinearOpMode {

        Hware robot = new Hware();

    /*
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor arm = null;
    private DcMotor lift = null;
    private Servo rightClaw = null;
    private Servo leftClaw = null;
    private Servo launcher = null;
    private Servo wrist = null;
    */

    private ColorSensor color1  = null;
    private  ColorSensor color2 = null;


    double movement;
    double rotation;
    double strafe;
    double index = -0.33;
    double index2 = 0.44;

    int x, y, z, y2, x2, z2;

    //    // 14524 Driver Controls
    private void driverControl() {
        movement = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotation = gamepad1.right_stick_x;

        robot.leftFront.setPower(-(movement + strafe + rotation)); //x > 0 p = forward
        robot.rightFront.setPower((movement - strafe - rotation)); // negative
        robot.leftBack.setPower((movement - strafe + rotation));
        robot.rightBack.setPower((movement + strafe - rotation)); // negative
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.leftFront  = hardwareMap.get(DcMotor.class, "LF");
        robot.rightFront = hardwareMap.get(DcMotor.class, "RF");
        robot.leftBack = hardwareMap.get(DcMotor.class, "LB");
        robot.rightBack = hardwareMap.get(DcMotor.class, "RB");

        robot.rightBack.setDirection(DcMotor.Direction.REVERSE);
        robot.rightFront.setDirection(DcMotor.Direction.REVERSE);


        color1 = hardwareMap.get(ColorSensor.class, "leftSensor");
        color2 = hardwareMap.get(ColorSensor.class, "rightSensor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips




//        boolean changeDrive = true;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if(opModeIsActive()){
            while (opModeIsActive()) {
                driverControl();

                lift();
                telemetry();
                x = color1.red();
                y = color1.blue();
                z = color1.green();
                x2 = color2.red();
                y2 = color2.blue();
                z2 = color2.green();
            }
        }
    }


    private void lift() {
        if (gamepad1.a) {
            reset();
        }

/*
Starting values:
wrist = 0.93
leftClaw = 1
rightClaw = -0.53

End values:
wrist = 0.85 --> 0.395
leftClaw = 0.73

*/


        if (gamepad2.a) {       //launcher down
            index -= 0.005;
            robot.launcher.setPosition(index);
        }
        if (gamepad2.y) {       //launcher up
            index += 0.005;
            robot.launcher.setPosition(0.5);
        }
        if (gamepad2.dpad_left) { //claw close
            robot.clawClose();
        }
        if (gamepad2.x) {
            robot.wrist.setPosition(0.85);
        }
        if (gamepad2.b) {
            robot.wrist.setPosition(0.5);
        }
        if (gamepad2.dpad_right) {  //claw open
            robot.clawOpen();
        }
        if ((gamepad2.dpad_up) && (robot.armNewTarget != 1074)) {  //Up
            robot.armEncode(1.4, false);
            robot.liftEncode(-2 / -gamepad2.right_stick_y);
            robot.wrist.setPosition(0.80);

        }
        if (gamepad2.dpad_down) {  //Down
            robot.lift.setTargetPosition(0);
            robot.lift.setPower(0.2);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            robot.arm.setTargetPosition(0);
            robot.arm.setPower(0.2);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armNewTarget = 0;

        }
    }




    public void reset() {
        robot.lift.setTargetPosition(0);
        robot.lift.setPower(0.2);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.arm.setTargetPosition(0);
        robot.arm.setPower(0.2);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightClaw.setPosition(-0.33);
        robot.leftClaw.setPosition(1);
        robot.launcher.setPosition(0.5);
        robot.wrist.setPosition(0.93);
        robot.armNewTarget = 0;
        robot.liftNewTarget = 0;
    }

    private void telemetry() {
        //telemetry.addData("FL Power", Double.parseDouble(JavaUtil.formatNumber(leftFront.getPower(), 2)));
        //telemetry.addData("BL Power", Double.parseDouble(JavaUtil.formatNumber(leftBack.getPower(), 2)));
        //telemetry.addData("FR Power", Double.parseDouble(JavaUtil.formatNumber(rightFront.getPower(), 2)));
        //telemetry.addData("BR Power", Double.parseDouble(JavaUtil.formatNumber(rightBack.getPower(), 2)));
        //    telemetry.addData("Lift 1 position", Double.parseDouble(JavaUtil.formatNumber(Lift1.getCurrentPosition(), 2)));
        //    telemetry.addData("Lift 2 position", Double.parseDouble(JavaUtil.formatNumber(Lift2.getCurrentPosition(), 2)));
        //    telemetry.addData("Angle Lift Position", Double.parseDouble(JavaUtil.formatNumber(ExtLift.getCurrentPosition(), 2)));
        telemetry.addData("leftClaw = ", index);
        telemetry.addData("leftSensor: ", x + " " + y + " " + z);
        telemetry.addData("rightSensor: ", x2 + " " + y2 + " " + z2);
        telemetry.addData("LF: ", robot.leftBack.getPower());
        //telemetry.addData("leftSensor: ", x2 + " " + y2 + " " + z2);

        telemetry.update();
    }
}


