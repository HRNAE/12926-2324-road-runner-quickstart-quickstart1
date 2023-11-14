    package org.firstinspires.ftc.teamcode;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.ColorSensor;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.Servo;

    import org.firstinspires.ftc.robotcore.external.JavaUtil;


@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class testOpMode extends LinearOpMode {

    // Declare OpMode members.
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
    private ColorSensor color1  = null;
    private  ColorSensor color2 = null;


    double movement;
    double rotation;
    double strafe;
    double ticks = 751.8;
    double newTarget;
    double  armNewTarget;
    double index = -0.33;
    double index2 = 0.44;

    int x, y, z, y2, x2, z2;

    //    // 14524 Driver Controls
    private void driverControl (boolean check) {
        movement = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotation = gamepad1.right_stick_x;

        if (check) {
            leftFront.setPower(movement + strafe + rotation); //x > 0 p = forward
            rightFront.setPower(-1 * (movement - strafe - rotation)); // negative
            leftBack.setPower(movement - strafe + rotation);
            rightBack.setPower(-1 * (movement + strafe - rotation)); // negative
        }
        else {
            runDriveTrain();
        }
    }

    private void runDriveTrain() {
        /*
        int ratio;
        movement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        if (gamepad1.right_bumper) {
            ratio = 2;
        } else { ratio = 1; }
        if (rotation > 0.1) {
            leftBack.setPower(movement/ ratio);
            leftFront.setPower(movement / -ratio);
            rightBack.setPower(movement / -ratio);
            rightFront.setPower(movement / ratio);
        } else if (rotation < -0.1) {
            leftBack.setPower(movement / -ratio);
            leftFront.setPower(movement/ ratio);
            rightBack.setPower(movement/ ratio);
            rightFront.setPower(movement / -ratio);
        } else {
            leftBack.setPower((movement + rotation) / 2 / ratio);
            leftFront.setPower((movement - rotation) / 2 / ratio);
            rightBack.setPower((movement - rotation) / 2 / ratio);
            rightFront.setPower((movement + rotation) / 2 / ratio);
        }

*/


        double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double direction = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        boolean precision = gamepad1.right_bumper;

        //INFO Increasing speed to a maximum of 1
        double lf = -1 * magnitude * Math.sin(direction + Math.PI) + rotation;
        double lb = -1 * magnitude * Math.cos(direction + Math.PI) + rotation;
        double rf = magnitude * Math.cos(direction + Math.PI) - rotation;
        double rb = magnitude * Math.sin(direction + Math.PI) - rotation;

//hgug

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

        telemetry.addData("magnitude: ", gamepad1.left_stick_y);
        telemetry.addData("direction: ", gamepad1.right_stick_x);
        telemetry.addData("ratio: ", ratio);
        telemetry.update();

    }


    // Define a variable for our color sensor





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

        leftBack.setDirection(DcMotor.Direction.REVERSE);


        arm = hardwareMap.get(DcMotor.class, "arm");
        lift = hardwareMap.get(DcMotor.class, "lift");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        launcher = hardwareMap.get(Servo.class, "planeLauncher");
        wrist = hardwareMap.get(Servo.class, "wrist");

        color1 = hardwareMap.get(ColorSensor.class, "leftSensor");
        color2 = hardwareMap.get(ColorSensor.class, "rightSensor");

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
                if (gamepad1.left_bumper) {
                    driverControl(true);
                }
                runDriveTrain();

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
            launcher.setPosition(index);
        }
        if (gamepad2.y) {       //launcher up
            index += 0.005;
            launcher.setPosition(0.5);
        }
        if (gamepad2.dpad_left) { //claw close
            leftClaw.setPosition(0);
            rightClaw.setPosition(0.695);
        }
        if (gamepad2.x) {
            wrist.setPosition(0.85);
        }
        if (gamepad2.b) {
            wrist.setPosition(0.5);
        }
        if (gamepad2.dpad_right) {  //claw open
            leftClaw.setPosition(0.5);
            rightClaw.setPosition(0.3);
        }
        if ((gamepad2.dpad_up) && (armNewTarget != 1074)) {  //Up
            armEncode(0.7);
            liftEncode(-0.5 / -gamepad2.right_stick_y);
            wrist.setPosition(0.80);

        }
        if (gamepad2.dpad_down) {  //Down
            lift.setTargetPosition(0);
            lift.setPower(0.2);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            arm.setTargetPosition(0);
            arm.setPower(0.2);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armNewTarget = 0;

        }
    }


    public void liftEncode(double turnage) {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticks / turnage;
        lift.setTargetPosition((int) newTarget);
        lift.setPower(-.4);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void armEncode(double turnage) {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armNewTarget = ticks / turnage;
        arm.setTargetPosition((int) armNewTarget);
        arm.setPower(-.1);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void reset() {
        lift.setTargetPosition(0);
        lift.setPower(0.2);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setTargetPosition(0);
        arm.setPower(0.2);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightClaw.setPosition(-0.33);
        leftClaw.setPosition(1);
        launcher.setPosition(0.5);
        wrist.setPosition(0.93);
        armNewTarget = 0;
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
        telemetry.addData("LF: ", leftBack.getPower());
        //telemetry.addData("leftSensor: ", x2 + " " + y2 + " " + z2);

        telemetry.update();
    }
}