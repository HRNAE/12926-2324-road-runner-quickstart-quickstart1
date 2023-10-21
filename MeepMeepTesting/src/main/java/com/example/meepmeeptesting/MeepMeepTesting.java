package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geo
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;


public class  MeepMeepTesting {
    public static void main(String args[]) {
        MeepMeep mm = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_ULTIMATE_GOAL_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(lf)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.TrajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                .forward(30)
                                .build()
                )
                .start();
    }
}
