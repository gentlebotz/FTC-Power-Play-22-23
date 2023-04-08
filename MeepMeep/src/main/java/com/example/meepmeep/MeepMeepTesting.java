package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

// Used to visualize & plan autonomous trajectories before testing on robot
public class MeepMeepTesting {

    private static float junctionOffset = 6;

    // Get junction coordinate
    private static Vector2d GetJunction(int x, int y) {
        return new Vector2d((-70 + (x * 23.33)), (-70 + (y * 23.33)));
    }

    // Junction overload with offset
    private static Vector2d GetJunction(int x, int y, int xOffset, int yOffset) {
        return new Vector2d((-70 + (x * 23.333)) + xOffset * junctionOffset, (-70 + (y * 23.333)) + yOffset * junctionOffset);
    }

    // Get tile coordinate
    private static Vector2d GetTile(int x, int y) {
        return new Vector2d((-58.333 + (x * 23.333)), (-58.333 + (y * 23.333)));
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity leftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-33, -61.25, Math.toRadians(270))) //Red F2 Starting position

                                // Drop preloaded cone
                                // Sliders up
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                    sliderRight.setPower(0.4);
//                                    sliderLeft.setPower(0.4);
//
//                                    sliderRight.setTargetPosition(highPole);
//                                    sliderLeft.setTargetPosition(highPole);
                                })

                                .lineToConstantHeading(new Vector2d(-33.5, -59))
                                .splineToConstantHeading(new Vector2d(-34, -47), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-35, 0), Math.toRadians(90)) // Move to F3
                                .lineToConstantHeading(new Vector2d(-35, -13)) // Move to F3
                                .lineToLinearHeading(new Pose2d(-4, -13, Math.toRadians(250))) // Move to E3
                                .lineToLinearHeading(new Pose2d(-15, -13, Math.toRadians(0)))

                                // Slider up, prepare outtake
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                                    intakeArmServoLeft.setPosition(0.2);
//                                    intakeArmServoRight.setPosition(0.2);
                                })

                                .waitSeconds(.4)

                                .lineToLinearHeading(new Pose2d(-4, -27.33, Math.toRadians(45))) // 45 deg hi approach

                                // Drop cone
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                                    intakeHand.setPosition(handOpenPos)
                                })

                                .waitSeconds(3)

                                // Sliders down
                                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
//                                    sliderRight.setPower(0.4);
//                                    sliderLeft.setPower(0.4);
//
//                                    sliderRight.setTargetPosition(lowPole);
//                                    sliderLeft.setTargetPosition(lowPole);
                                })

                                .lineToLinearHeading(new Pose2d(-11.67, -35, Math.toRadians(90))) // Reverse approach back to E3
                                .lineToConstantHeading(new Vector2d(-11.67, -12.2)) // Move to D3 and rotate for cycles

                                // Cycles
                                // .lineToLinearHeading(new Pose2d(-35, -11.67, Math.toRadians(-180))) // Move to B3 and rotate for cycles
                                .turn(Math.toRadians(90))

                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                                    intakeArmServoLeft.setPosition(intakeArmPos);
//                                    intakeArmServoRight.setPosition(intakeArmPos);
                                })

                                .lineToConstantHeading(new Vector2d(-58.33, -12.2)) // Move to cone stack D1

                                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
//                                    sliderRight.setPower(0.2);
//                                    sliderLeft.setPower(0.2);
//
//                                    sliderRight.setTargetPosition(lowPole - 100);
//                                    sliderLeft.setTargetPosition(lowPole - 100);
                                })

                                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
//                                    intakeHand.setPosition(handClosedPos)
//                                    sliderRight.setTargetPosition(lowPole + 200);
////                                  sliderLeft.setTargetPosition(lowPole + 200);
                                })

                                .waitSeconds(2)

                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
//                                    sliderRight.setPower(0.4);
//                                    sliderLeft.setPower(0.4);
//
//                                    sliderRight.setTargetPosition(highPole);
//                                    sliderLeft.setTargetPosition(highPole);
                                })

                                .lineToConstantHeading(new Vector2d(-35, -12.2)) // Move to D2
                                .lineToLinearHeading(new Pose2d(-29.33, -6, Math.toRadians(225))) // 45 deg hi approach

                                // Drop cone
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                                    intakeHand.setPosition(handOpenPos)
                                })

                                .waitSeconds(2)

                                // Sliders down
                                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
//                                    sliderRight.setPower(0.4);
//                                    sliderLeft.setPower(0.4);
//
//                                    sliderRight.setTargetPosition(lowPole);
//                                    sliderLeft.setTargetPosition(lowPole);
                                })

                                .lineToLinearHeading(new Pose2d(-35, -12.2, Math.toRadians(-180))) // Reverse approach back to D2

                                // Sliders down and setup for driver-controlled
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                                    sliderRight.setPower(0.4);
//                                    sliderLeft.setPower(0.4);
//
//                                    sliderRight.setTargetPosition(0);
//                                    sliderLeft.setTargetPosition(0);
//
//                                    intakeArmServoLeft.setPosition(intakeArmPos);
//                                    intakeArmServoRight.setPosition(intakeArmPos);
//                                    intakeHand.setPosition(handOpenPos)
                                })

                                // Parking
                                .lineToConstantHeading(new Vector2d(-58.33, -12.2)) // Location 1
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(-35, -12.2)) // Location 2
                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(-11.67, -12.2)) // Location 3

                                .build()
                );

        // RIGHT TRAJECTORY
        RoadRunnerBotEntity rightBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(35, -58.33, Math.toRadians(270))) //Red F2 Starting position

                                        // Drop preloaded cone
                                        .lineToConstantHeading(new Vector2d(19, -58.33)) // Move to F3
                                        .splineToConstantHeading(new Vector2d(11.67, -40), Math.toRadians(90)) // Move to E3

                                        // Slider up, prepare outtake
                                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                                    sliderRight.setPower(0.5);
//                                    sliderLeft.setPower(0.5);
//
//                                    sliderRight.setTargetPosition(800);
//                                    sliderLeft.setTargetPosition(800);
//
//                                    intakeArmServo.setPosition(0);
                                        })

                                        .lineToLinearHeading(new Pose2d(6, -29.33, Math.toRadians(320))) // 45 deg hi approach

                                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//                                    intakeWheelServo.setPower(-1);
                                        })

                                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                                    intakeWheelServo.setPower(0);
                                        })

                                        .waitSeconds(3)

                                        .lineToLinearHeading(new Pose2d(11.67, -35, Math.toRadians(0))) // Reverse approach back to E3
                                        .lineToLinearHeading(new Pose2d(11.67, -11.67, Math.toRadians(0))) // Move to D3 and rotate for cycles

                                        // Cycles
                                        // .lineToLinearHeading(new Pose2d(35, -11.67, Math.toRadians(180))) // Move to B3 and rotate for cycles
                                        .lineToConstantHeading(new Vector2d(58.33, -11.67)) // Move to cone stack D1
                                        .lineToConstantHeading(new Vector2d(35, -11.67)) // Move to D2
                                        .lineToLinearHeading(new Pose2d(29.33, -6, Math.toRadians(-45))) // 45 deg hi approach
                                        .lineToLinearHeading(new Pose2d(35, -11.67, Math.toRadians(0))) // Reverse approach back to D2

                                        // Parking
                                        .lineToConstantHeading(new Vector2d(58.33, -11.67)) // Location 1
                                        .waitSeconds(1)
                                        .lineToConstantHeading(new Vector2d(35, -11.67)) // Location 2
                                        .waitSeconds(1)
                                        .lineToConstantHeading(new Vector2d(11.67, -11.67)) // Location 3

                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(leftBot)
                .addEntity(rightBot)
                .start();
    }
}