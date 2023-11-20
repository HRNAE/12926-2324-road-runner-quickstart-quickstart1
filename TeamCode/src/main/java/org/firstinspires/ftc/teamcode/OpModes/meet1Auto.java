package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareMap.Hware;

@Autonomous(name = "meet1Auto")
 public class meet1Auto extends LinearOpMode {
     Hware robot = new Hware();
     ColorSensor leftSensor = null;
     ColorSensor rightSensor = null;
     int leftD, rightD = 0;

     @Override
     public void runOpMode() throws InterruptedException {
         waitForStart();
         drive(-1000, -1000, 0.3);
         drive(5000, 5000, 0.3);
     }

     private void drive(double left, double right, double speed) {
         leftD += left;
         rightD += right;

         //leftFront.setTargetPosition(left);
         robot.leftBack.setTargetPosition((int)left);
         //rightFront.setTargetPosition(right);
         robot.rightBack.setTargetPosition((int)right);

         //leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         //rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         //leftFront.setPower(speed - 0.05);
         robot.leftBack.setPower(speed - 0.05);
         //rightFront.setPower(speed);
         robot.rightBack.setPower(speed);

         robot.leftClaw.setPosition(0);
         robot.rightClaw.setPosition(0.695);
         if (getRuntime() > 7) {
             robot.wrist.setPosition(0.45);
         }
         if (getRuntime() > 8) {
             robot.leftClaw.setPosition(0.5);
             robot.rightClaw.setPosition(0.3);
         }

         while (opModeIsActive()) { //If the motors are running, then it wont move other motors
         }
     }
}


