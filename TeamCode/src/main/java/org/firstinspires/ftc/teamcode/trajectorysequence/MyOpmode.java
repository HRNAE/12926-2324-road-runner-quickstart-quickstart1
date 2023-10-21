package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


public class MyOpmode extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPosition = new Pose2d(-36,54, Math.toRadians(180));
        drive.setPoseEstimate(startPosition);

        Trajectory mytraj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .forward(5)
                .build();
        //Trajectory mytraj2 = drive.trajectoryBuilder(new Pose2d())
        Trajectory mytraj2 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .forward(5)
                .build();
        Trajectory mytraj3 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .forward(5)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(mytraj1);
    }
}
