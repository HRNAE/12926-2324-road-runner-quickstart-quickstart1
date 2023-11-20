package com.example.meepmeeptesting1;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 60, Math.toRadians(180), Math.toRadians(180), 13.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                        .forward(24)
                        //drop pixel one
                        .lineToSplineHeading(new Pose2d(48, -34, Math.toRadians(180)))
                        .waitSeconds(1)
                        .forward(108)
                        .back(6)
                        .waitSeconds(1)
                        //pick up claw
                        .back(102)
                        .waitSeconds(1)
                        //drop pixels
                        .forward(102)
                        .waitSeconds(1)
                        .back(102)
                        .waitSeconds(1)

                        .forward(102)
                        .waitSeconds(1)
                        .back(102)
                        .waitSeconds(1)


                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

