
    package org.firstinspires.ftc.teamcode.OpModes;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.ColorSensor;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.Servo;



    @TeleOp(name="meet1OpMode", group="Linear OpMode")
public class meet1OpMode extends LinearOpMode {

    private ColorSensor color1  = null;
    private ColorSensor color2 = null;
    public DcMotor rightFront = null;
    public DcMotor leftFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftBack = null;
    public DcMotor arm, lift;
    public Servo rightClaw = null;
    public Servo launcher = null;
    public Servo leftClaw = null;
    public Servo wristLeft = null;
    public Servo wristRight = null;

    double movement;
    double rotation;
    double strafe;
    double index = -0.33;
    double armNewTarget, liftNewTarget;
    double ticks = 751.8;
    int x, y, z, y2, x2, z2;

    //    // 14524 Driver Controls
    private void driverControl() {
        double ratio = 1;
        boolean precision =  gamepad1.right_bumper;
        movement = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotation = gamepad1.right_stick_x;

        leftFront.setPower(-1 * ratio * (movement + strafe + rotation));
        rightFront.setPower(ratio * (movement - strafe - rotation));
        leftBack.setPower(ratio * (movement - strafe + rotation));
        rightBack.setPower(ratio * (movement + strafe - rotation));

        if (precision) {
            ratio = 0.5;
        } else {
            ratio = 1.0;
        }
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

        arm = hardwareMap.get(DcMotor.class, "arm");
        lift = hardwareMap.get(DcMotor.class, "lift");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        wristRight = hardwareMap.get(Servo.class, "wristRight");

        launcher = hardwareMap.get(Servo.class, "planeLauncher");

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);


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
            //reset();
        }
        if (gamepad2.a) {       //launcher down
            index -= 0.005;
            launcher.setPosition(index);
        }
        if (gamepad2.y) {       //launcher up
            index += 0.005;
            launcher.setPosition(0.5);
        }
        if (gamepad1.dpad_left) { //claw close
            clawClose();
        }
        if (gamepad1.dpad_right) {  //claw open
            clawOpen();
        }

        if (gamepad2.x) {
            wristUp();
        }
        if (gamepad2.b) {
            wristDown();
        }
        wristRight.setPosition(-gamepad2.left_stick_y);
        if ((gamepad2.dpad_up) && (armNewTarget != 1074)) {  //Up
            armEncode(.75, false);
            liftEncode(-2 / -gamepad2.right_stick_y, false);
            wristUp();
        }
        if (gamepad2.dpad_down) {  //Down
            liftEncode(1, true);
            armEncode(-.75, false);
            armNewTarget = 0;
            liftNewTarget = 0;

        }

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
        wristRight.setPosition(0.8);
        wristLeft.setPosition(0.8);
        armNewTarget = 0;
        liftNewTarget = 0;
    }
        private void clawClose() {
            leftClaw.setPosition(0.2);
            rightClaw.setPosition(0.55);
        }

        private void clawOpen() {
            leftClaw.setPosition(0.65);
            rightClaw.setPosition(0.1);
        }

        public void wristDown() { wristLeft.setPosition(1);}

        public void wristUp() {
            wristLeft.setPosition(0);
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
                lift.setPower(-.7);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        public void armEncode(double turnage, boolean reset) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armNewTarget = ticks / turnage;
                arm.setTargetPosition((int) armNewTarget);
                arm.setPower(.2);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    private void telemetry() {
        //telemetry.addData("FL Power", Double.parseDouble(JavaUtil.formatNumber(leftFront.getPower(), 2)));
        //telemetry.addData("BL Power", Double.parseDouble(JavaUtil.formatNumber(leftBack.getPower(), 2)));
        //telemetry.addData("FR Power", Double.parseDouble(JavaUtil.formatNumber(rightFront.getPower(), 2)));
        //telemetry.addData("BR Power", Double.parseDouble(JavaUtil.formatNumber(rightBack.getPower(), 2)));
        //    telemetry.addData("Lift 1 position", Double.parseDouble(JavaUtil.formatNumber(Lift1.getCurrentPosition(), 2)));
        //    telemetry.addData("Lift 2 position", Double.parseDouble(JavaUtil.formatNumber(Lift2.getCurrentPosition(), 2)));
        //    telemetry.addData("Angle Lift Position", Double.parseDouble(JavaUtil.formatNumber(ExtLift.getCurrentPosition(), 2)));
        telemetry.addData("wrist = ", -gamepad2.left_stick_y);
        telemetry.addData("leftSensor: ", x + " " + y + " " + z);
        telemetry.addData("rightSensor: ", x2 + " " + y2 + " " + z2);
        //telemetry.addData("LF: ", leftBack.getPower());
        //telemetry.addData("leftSensor: ", x2 + " " + y2 + " " + z2);

        telemetry.update();
    }
}


