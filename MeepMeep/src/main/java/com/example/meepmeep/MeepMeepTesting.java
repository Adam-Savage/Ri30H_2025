package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(40, -60, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(-58, -58), Math.toRadians(225))
                .setReversed(false)
                .splineTo(new Vector2d(0, -32), Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(-58, -58), Math.toRadians(225))
                .setReversed(false)
                .splineTo(new Vector2d(-26, -10), 0)
                .splineTo(new Vector2d(-23, -10), 0)
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}