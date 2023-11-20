package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMap.Hware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "meet2Auto")
public class MyOpmode extends LinearOpMode {

    Hware robot = new Hware();
    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPosition = new Pose2d(-36,54, Math.toRadians(180));
        drive.setPoseEstimate(startPosition);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPosition)

                .addDisplacementMarker(() -> {
                    robot.wristDown();
                })
                .forward(24)
                .addDisplacementMarker(() -> {
                    robot.leftClaw.setPosition(0.5);
                })

                .lineToSplineHeading(new Pose2d(48, -34, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    robot.clawClose();
                    robot.wristUp();
                    robot.liftEncode(0.25, false);
                    robot.armEncode(1, false);
                    robot.clawOpen();
                    robot.liftEncode(0.25, true);
                    robot.armEncode(1, true);
                })

                .forward(108)
                .back(6)
                .addDisplacementMarker(() -> {
                    robot.clawOpen();
                    robot.wristDown();
                    robot.clawClose();
                    robot.wristUp();
                })
                .back(102)
                .addDisplacementMarker(() -> {
                    robot.clawClose();
                    robot.wristUp();
                    robot.liftEncode(0.45, false);
                    robot.armEncode(1, false);
                    robot.clawOpen();
                    robot.liftEncode(0.25, true);
                    robot.armEncode(1, true);
                })


                .forward(102)
                .addDisplacementMarker(() -> {
                    robot.clawOpen();
                    robot.wristDown();
                    robot.clawClose();
                    robot.wristUp();
                })
                .back(102)
                .addDisplacementMarker(() -> {
                    robot.clawClose();
                    robot.wristUp();
                    robot.liftEncode(0.65, false);
                    robot.armEncode(1, false);
                    robot.clawOpen();
                    robot.liftEncode(0.25, true);
                    robot.armEncode(1, true);
                })

                .forward(102)
                .waitSeconds(1)
                .back(102)
                .waitSeconds(1)

                .forward(102)
                .waitSeconds(1)
                .back(102)
                .waitSeconds(1)

                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajSeq);
    }
}
