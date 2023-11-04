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

 //   private DcMotor arm = null;
    private DcMotor lift = null;
    double movement;
    double rotation;
    double strafe;
    double ticks = 751.8;
    double newTarget;

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
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
   //     arm = hardwareMap.get(DcMotor.class, "arm");
        lift = hardwareMap.get(DcMotor.class, "lift");
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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
                lift();
                telemetry();
            }
        }
    }


    private void lift() {
        if (gamepad1.a) {
            tracker();
        }
        if (gamepad1.b) {
            encoder(-0.5);
        }
    }

    private void encoder(double turnage) {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticks / turnage;
        lift.setTargetPosition((int) newTarget);
        lift.setPower(-.5);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void tracker() {
        lift.setTargetPosition(0);
        lift.setPower(0.5);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
}



