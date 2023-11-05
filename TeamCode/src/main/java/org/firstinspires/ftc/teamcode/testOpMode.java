    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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


    double movement;
    double rotation;
    double strafe;
    double ticks = 751.8;
    double newTarget;
    double  armNewTarget;
    double index = -0.33;

    // 12926 Driver Controls
    private void runDriveTrain() {
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

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hardwareMap.get(DcMotor.class, "arm");
        lift = hardwareMap.get(DcMotor.class, "lift");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        launcher = hardwareMap.get(Servo.class, "planeLauncher");
        wrist = hardwareMap.get(Servo.class, "wrist");


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

                runDriveTrain();
                //resetLift();
                //cascadinglift();
                // test();
                // dumpMovement();
                lift();
                telemetry();
            }
        }
    }


    private void lift() {
        if (gamepad1.a) {
            reset();
        }
        if (gamepad1.b) {
            liftEncode(-1);
        }
        if (gamepad1.x) {
            armEncode(-5);
        }
        if (gamepad1.dpad_down) {
            index -= 0.005;
            launcher.setPosition(index);
/*
Starting values:
wrist = 0.93
leftClaw = 1
*/
        }
        if (gamepad1.dpad_up) {
            index += 0.005;
            launcher.setPosition(index);
        }
 /*
Starting values:
wrist = 0.445
leftClaw = 0.73

*/
    }

    public void liftEncode(double turnage) {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticks / turnage;
        lift.setTargetPosition((int) newTarget);
        lift.setPower(-.2);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void armEncode(double turnage) {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armNewTarget = ticks / turnage;
        arm.setTargetPosition((int) newTarget);
        arm.setPower(-.2);
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
    }

    private void telemetry() {
        telemetry.addData("FL Power", Double.parseDouble(JavaUtil.formatNumber(leftFront.getPower(), 2)));
        telemetry.addData("BL Power", Double.parseDouble(JavaUtil.formatNumber(leftBack.getPower(), 2)));
        telemetry.addData("FR Power", Double.parseDouble(JavaUtil.formatNumber(rightFront.getPower(), 2)));
        telemetry.addData("BR Power", Double.parseDouble(JavaUtil.formatNumber(rightBack.getPower(), 2)));
        //       telemetry.addData("Lift 1 position", Double.parseDouble(JavaUtil.formatNumber(Lift1.getCurrentPosition(), 2)));
        //      telemetry.addData("Lift 2 position", Double.parseDouble(JavaUtil.formatNumber(Lift2.getCurrentPosition(), 2)));
        //    telemetry.addData("Angle Lift Position", Double.parseDouble(JavaUtil.formatNumber(ExtLift.getCurrentPosition(), 2)));
        telemetry.addData("index = ", index);
        telemetry.addData("precision: ", gamepad1.right_bumper);
        telemetry.update();
    }
}