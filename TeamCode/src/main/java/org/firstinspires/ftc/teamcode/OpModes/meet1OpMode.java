package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OpModes.Hware;

@TeleOp(name="meet2OpMode", group ="Linear Opmode")
public class meet1OpMode extends LinearOpMode {
    double movement, strafe, rotation;
    Hware robot = new Hware(hardwareMap);


    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                chassisControl();
            }
        }
    }

    private void chassisControl() {
        movement = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotation = gamepad1.right_stick_x;

        robot.leftFront.setPower(-(movement + strafe + rotation)); //x > 0 p = forward
        robot.rightFront.setPower((movement - strafe - rotation)); // negative
        robot.leftBack.setPower((movement - strafe + rotation));
        robot.rightBack.setPower((movement + strafe - rotation)); // negative
    }
}


