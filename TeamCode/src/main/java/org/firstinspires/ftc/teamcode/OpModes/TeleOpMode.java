
    package org.firstinspires.ftc.teamcode.OpModes;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.ColorSensor;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.Servo;

    import org.firstinspires.ftc.robotcore.external.JavaUtil;


    @TeleOp(name="TeleOpMode", group="Linear OpMode")
public class TeleOpMode extends LinearOpMode {
    Hware robot;

    double movement;
    double rotation;
    double strafe;
    double index = -0.33;
    double armNewTarget;
    double liftNewTarget = 100;
    double ticks = 751.8;
    public double ratio = 1;


    private void driverControl() {
        ratio = 1;
        boolean precision =  gamepad1.right_bumper;
        movement = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotation = gamepad1.right_stick_x;

        robot.leftFront.setPower(ratio * (movement + strafe + rotation));
        robot.rightFront.setPower(ratio * (movement - strafe - rotation));
        robot.leftBack.setPower(ratio * (movement - strafe + rotation));
        robot.rightBack.setPower(ratio * (movement + strafe - rotation));

        if (precision) {
            ratio = 0.5;
        } else {
            ratio = 1.0;
        }
    }

    @Override
    public void runOpMode() {
        robot = new Hware(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phaone).


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips




//        boolean changeDrive = true;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        liftEncode(0, true);
        if(opModeIsActive()){
            while (opModeIsActive()) {
                driverControl();

                lift();
                telemetry();
                if (gamepad1.right_bumper) {
                    ratio = 0.5;
                } else {
                    ratio = 1;
                }
            }
        }
    }

    private void lift() {
        if (gamepad2.a) {       //launcher down
            index -= 0.005;
            robot.launcher.setPosition(index);
        }
        if (gamepad2.y) {       //launcher up
            index += 0.005;
            robot.launcher.setPosition(0.5);
        }
        if (gamepad2.dpad_left) { //claw close
            clawClose();
        }
        if (gamepad2.dpad_right) {  //claw open
            clawOpen();
        }
        if (gamepad2.x) {
            wristUp();
            //wristRight.setPosition(0);
        }
        if (gamepad2.b) {
            wristDown();
            //wristRight.setPosition(1);
        }
        robot.wristRight.setPosition(-gamepad2.right_stick_y/2);

        if ((gamepad2.dpad_up) && (liftNewTarget > -1074)) {  //Up
            liftEncode(-2, false);
            armEncode(0.75, false);
            wristUp();
        }
        if (gamepad2.dpad_down) {  //Down
            liftEncode(2, true);
            armEncode(-.75, false);
            armNewTarget = 0;
            liftNewTarget = 0;

        }
    }
//    public void reset() {
//        robot.lift.setTargetPosition(0);
//        robot.lift.setPower(0.2);
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.arm.setTargetPosition(0);
//        robot.arm.setPower(0.2);
//        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        rightClaw.setPosition(-0.33);
//        leftClaw.setPosition(1);
//        launcher.setPosition(0.5);
//        wristRight.setPosition(0.8);
//        wristLeft.setPosition(0.8);
//        armNewTarget = 0;
//        liftNewTarget = 0;
//    }
        private void clawClose() {
            robot.leftClaw.setPosition(0.2);
            robot.rightClaw.setPosition(0.55);
        }

        private void clawOpen() {
            robot.leftClaw.setPosition(0.55);
            robot.rightClaw.setPosition(0.1);
        }

        public void wristDown() {
            robot.wristLeft.setPosition(1);
            robot.wristRight.setPosition(0);
        }

        public void wristUp() {
            robot.wristLeft.setPosition(0 );
            robot.wristRight.setPosition(1);
        }

        public void liftEncode(double turnage, boolean reset) {
            robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftNewTarget = ticks * turnage;
            robot.lift.setTargetPosition((int) liftNewTarget);
            robot.lift.setPower(.5);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        public void armEncode(double turnage, boolean reset) {
            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armNewTarget = ticks / turnage;
            robot.arm.setTargetPosition((int) armNewTarget);
            robot.arm.setPower(.3);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    private void telemetry() {
        telemetry.addData("FL Power", Double.parseDouble(JavaUtil.formatNumber(robot.leftFront.getPower(), 2)));
        telemetry.addData("BL Power", Double.parseDouble(JavaUtil.formatNumber(robot.leftBack.getPower(), 2)));
        telemetry.addData("FR Power", Double.parseDouble(JavaUtil.formatNumber(robot.rightFront.getPower(), 2)));
        telemetry.addData("BR Power", Double.parseDouble(JavaUtil.formatNumber(robot.rightBack.getPower(), 2)));
        telemetry.addData("liftNewTarget", liftNewTarget);
        telemetry.addData("wrist = ", robot.wristLeft.getPosition());
        telemetry.update();
    }
}


