package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "AutoPP", group = "AutoRR")
public class AutoPowerPlay extends LinearOpMode {

    /*
    Robot Vars
     */
    // Wheels
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightRear = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;

    // Slider, Intake
    private DcMotor sliderLeft = null;
    private DcMotor sliderRight = null;
    private TouchSensor sliderLimitSwitch = null;
    private Servo intakeArmServoLeft = null;
    private Servo intakeArmServoRight = null;
    private Servo intakeHand = null;

    // Variables
    private double power = 1;
    private double driveSpeed = .5;
    private double drivePower = .5;
    private double turboPower = 1;
    private boolean turboStop = true;
    private boolean turbo = false;

    private boolean handClosed;
    private boolean xPressed;
    private boolean yPressed;

    // Lift variables
    private int target = 0;
    private int sliderSpeed = 180;
    private int current;
    private int incr;
    private int maxHeight = 6000;
    private int minHeight = -10;

    private enum LiftState {
        LIFT_START,
        LIFT_LOW,
        LIFT_MID,
        LIFT_HIGH
    }

    LiftState liftState = LiftState.LIFT_START;

    private int lowPole = 1630;
    private int midPole = 3000;
    private int highPole = 4100;
    private int stackHight = 1000;

    private double intakeArmPickupPosition = 0.8;
    private double intakeArmMidPosition = 0.5;
    private double intakeArmFrontDropPosition = 0.2;
    private double intakeArmDropPosition = 0;

    private boolean initHandClosed = false;
    private boolean initHandPressed = false;
    private double handOpenPos = 0.29;
    private double handClosedPos = 0.1;

    private enum ArmState {
        ARM_INTAKE,
        ARM_MID_TO_DROP,
        ARM_MID_TO_INTAKE,
        ARM_DROP
    }

    private boolean aPressed = false;

    ArmState armState = ArmState.ARM_MID_TO_DROP;

    public static double mapRange(double a1, double a2, double b1, double b2, double s) {
        return b1 + ((s - a1) * (b2 - b1)) / (a2 - a1);
    }

    private boolean StartLeft = true;

    /*
    OpenCV Vars
    */
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution

    @Override
    public void runOpMode() throws InterruptedException {
        // Telemetry
        telemetry.addData("Status: ", "Busy");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        sliderLeft = hardwareMap.get(DcMotor.class, "sliderLeft");
        sliderRight = hardwareMap.get(DcMotor.class, "sliderRight");
        sliderLimitSwitch = hardwareMap.get(TouchSensor.class, "sliderLimitSwitch");
        intakeArmServoLeft = hardwareMap.get(Servo.class, "intakeArmServoL");
        intakeArmServoRight = hardwareMap.get(Servo.class, "intakeArmServoR");
        intakeHand = hardwareMap.get(Servo.class, "intakeHand");

        //  Motor Direction
        sliderLeft.setDirection(DcMotor.Direction.FORWARD);
        sliderRight.setDirection(DcMotor.Direction.REVERSE);
        intakeArmServoRight.setDirection(Servo.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        //Encoders
        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /*
        Initialize EasyOpenCV
         */

        int detectedID = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(CAMERA_WIDTH,CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        /*
        Initialize RoadRunner
         */

        Pose2d startPoseLeft = new Pose2d(-35, -61.25, Math.toRadians(90));
        Pose2d startPoseRight = new Pose2d(35, -61.25, Math.toRadians(90));

        /*
        Left side trajectories
         */
        TrajectorySequence trajLeft = drive.trajectorySequenceBuilder(startPoseLeft)
                // Drop preloaded cone
                // Sliders up
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                    sliderRight.setPower(0.7);
                                    sliderLeft.setPower(0.7);

                                    sliderRight.setTargetPosition(highPole);
                                    sliderLeft.setTargetPosition(highPole);
                })

                .lineToConstantHeading(new Vector2d(-13, -58.33)) // Move to F3
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(-15.5, -39.7), Math.toRadians(90)) // Move to E3

                // Slider up, prepare outtake
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    intakeArmServoLeft.setPosition(intakeArmFrontDropPosition);
                                    intakeArmServoRight.setPosition(intakeArmFrontDropPosition);
                })

                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(-8.7, -28.5, Math.toRadians(63))) // 63 deg hi approach

                // Drop cone
                .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                                    intakeHand.setPosition(handOpenPos);
                })

                .waitSeconds(.6)

                // Sliders down
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                                    sliderRight.setPower(0.6);
                                    sliderLeft.setPower(0.6);

                                    sliderRight.setTargetPosition(lowPole);
                                    sliderLeft.setTargetPosition(lowPole);
                })

                .lineToLinearHeading(new Pose2d(-17.5, -40.5, Math.toRadians(90))) // Reverse approach back to E3
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-15, -11)) // Move to D3 and rotate for cycles

                // Cycles
                .turn(Math.toRadians(90))

//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                                    intakeArmServoLeft.setPosition(intakeArmDropPosition);
//                                    intakeArmServoRight.setPosition(intakeArmDropPosition);
//                                    intakeHand.setPosition(handOpenPos);
//                })
//
//                .lineToConstantHeading(new Vector2d(-63, -13)) // Move to cone stack D1
//
//                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
//                                    sliderRight.setPower(0.3);
//                                    sliderLeft.setPower(0.3);
//
//                                    sliderRight.setTargetPosition(stackHight);
//                                    sliderLeft.setTargetPosition(stackHight);
//                })
//
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                                    intakeHand.setPosition(handClosedPos);
//                                    sliderRight.setTargetPosition(lowPole + 200);
//                                    sliderLeft.setTargetPosition(lowPole + 200);
//                })
//
//                .waitSeconds(1.15)
//
//                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
//                                    sliderRight.setPower(0.7);
//                                    sliderLeft.setPower(0.7);
//
//                                    sliderRight.setTargetPosition(highPole);
//                                    sliderLeft.setTargetPosition(highPole);
//
//                                    intakeArmServoLeft.setPosition(intakeArmPickupPosition);
//                                    intakeArmServoRight.setPosition(intakeArmPickupPosition);
//                })
//
//                .lineToConstantHeading(new Vector2d(-35, -12.2)) // Move to D2
//                .lineToLinearHeading(new Pose2d(-29.33, -6, Math.toRadians(225))) // 45 deg hi approach
//
//                // Drop cone
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                    intakeHand.setPosition(handOpenPos);
//                })
//
//                .waitSeconds(.75)
//
//                // Sliders down
//                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
//                                    sliderRight.setPower(0.7);
//                                    sliderLeft.setPower(0.7);
//
//                                    sliderRight.setTargetPosition(lowPole);
//                                    sliderLeft.setTargetPosition(lowPole);
//
//                                    intakeHand.setPosition(handClosedPos);
//
//                                    intakeArmServoLeft.setPosition(intakeArmDropPosition);
//                                    intakeArmServoRight.setPosition(intakeArmDropPosition);
//                })
//
//                .lineToLinearHeading(new Pose2d(-35, -12.2, Math.toRadians(-180))) // Reverse approach back to D2

                // Sliders down and setup for driver-controlled
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    sliderRight.setPower(0.6);
                                    sliderLeft.setPower(0.6);

                                    sliderRight.setTargetPosition(0);
                                    sliderLeft.setTargetPosition(0);

                                    intakeArmServoLeft.setPosition(100);
                                    intakeArmServoRight.setPosition(100);
                                    intakeHand.setPosition(handOpenPos);
                })

                .build();

        TrajectorySequence trajRight = drive.trajectorySequenceBuilder(startPoseLeft)
                // Drop preloaded cone
                .lineToConstantHeading(new Vector2d(19, -58.33)) // Move to F3
                .splineToConstantHeading(new Vector2d(11.67, -40), Math.toRadians(90)) // Move to E3

                // Slider up, prepare outtake
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    sliderRight.setPower(0.5);
                    sliderLeft.setPower(0.5);

                    sliderRight.setTargetPosition(highPole);
                    sliderLeft.setTargetPosition(highPole);

                    intakeArmServoLeft.setPosition(0.2);
                })

                .lineToLinearHeading(new Pose2d(6, -29.33, Math.toRadians(320))) // 45 deg hi approach

                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intakeHand.setPosition(handOpenPos);
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

                .build();
        /*
        Right Side Trajectories
         */


        /*
        Park trajectories
         */

        TrajectorySequence parkLeft1 = drive.trajectorySequenceBuilder(trajLeft.end())
                .lineToConstantHeading(new Vector2d(-63, -11.5)) // Location 1
                .waitSeconds(1)
                .build();

        TrajectorySequence parkLeft2 = drive.trajectorySequenceBuilder(trajLeft.end())
                .lineToConstantHeading(new Vector2d(-41, -12))
                .waitSeconds(1) // Location 2
                .build();

        TrajectorySequence parkLeft3 = drive.trajectorySequenceBuilder(trajLeft.end())
                //.lineToConstantHeading(new Vector2d(-11.67, -11.67)) // Location 3
                .waitSeconds(1)
                .build();

        TrajectorySequence parkRight3 = drive.trajectorySequenceBuilder(trajRight.end())
                .lineToConstantHeading(new Vector2d(58.33, -11.67)) // Location 3
                .waitSeconds(1)
                .build();

        TrajectorySequence parkRight2 = drive.trajectorySequenceBuilder(trajRight.end())
                .waitSeconds(1) // Location 2
                .build();

        TrajectorySequence parkRight1 = drive.trajectorySequenceBuilder(trajRight.end())
                .waitSeconds(2)
                .lineToConstantHeading(new Vector2d(11.67, -11.67)) // Location 1
                .waitSeconds(1)
                .build();

        drive.setPoseEstimate(startPoseLeft);

        // Pickup preloaded cone
//        intakeHand.setPosition(handClosedPos);
//        sleep(700);
//        intakeArmServoLeft.setPosition(intakeArmMidPosition);
//        intakeArmServoRight.setPosition(intakeArmMidPosition);
//        sleep(1000);

        intakeArmServoRight.setPosition(intakeArmFrontDropPosition);
        intakeArmServoLeft.setPosition(intakeArmFrontDropPosition);

        // Slider calibration
        sliderRight.setTargetPosition(400);
        sliderLeft.setTargetPosition(400);

        sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setPower(0.5);
        sliderLeft.setPower(0.5);

        while (Math.abs(400 - sliderLeft.getCurrentPosition()) >= 10 && !isStopRequested()) {
            // Move slider up to start calibration
        }

        sliderRight.setPower(0);
        sliderLeft.setPower(0);

        // Move slider down until it reaches limit switch
        sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!sliderLimitSwitch.isPressed() && sliderRight.getCurrentPosition() >= -400 && !isStopRequested()) {
            sliderLeft.setPower(-0.2);
            sliderRight.setPower(-0.2);
        }

        sliderLeft.setPower(0);
        sliderRight.setPower(0);

        sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sliderRight.setTargetPosition(0);
        sliderLeft.setTargetPosition(0);

        sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.setMsTransmissionInterval(50);
        intakeHand.setPosition(handOpenPos);

        //intakeArmServo.setPosition(intakeArmMidPosition);
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        while (!isStarted() && !isStopRequested())
        {
            // Select starting configuration
            if (gamepad2.dpad_left && !StartLeft) {
                StartLeft = true;
                drive.setPoseEstimate(startPoseLeft);
            } else if (gamepad2.dpad_right && StartLeft) {
                StartLeft = false;
                drive.setPoseEstimate(startPoseRight);
            }

            telemetry.addData("Starting location: ", StartLeft ? "Left side" : "Right side");
            
            // Setup servo for pre-loaded cone in init
            if (gamepad2.a) {
                intakeArmServoLeft.setPosition(intakeArmMidPosition);
                intakeArmServoRight.setPosition(intakeArmMidPosition);
                intakeHand.setPosition(handClosedPos);
            } else if (gamepad2.b) {
                intakeArmServoLeft.setPosition(intakeArmFrontDropPosition);
                intakeArmServoRight.setPosition(intakeArmFrontDropPosition);
            }

            if (gamepad2.x && !initHandPressed) {
                initHandPressed = true;
                initHandClosed = !initHandClosed;
                intakeHand.setPosition(initHandClosed ? handClosedPos : handOpenPos);
            } else if (!gamepad2.x) {
                initHandPressed = false;
            }

            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame
            if(detections != null)
            {
//                telemetry.addData("FPS", camera.getFps());
//                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
//                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // We don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                        detectedID = detection.id;
//                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                    }
                }
            }

            telemetry.addLine(String.format("\nDetected tag ID=%d", detectedID));
            telemetry.update();
        }

        // Wait until start button is pressed
        waitForStart();

        if (!isStopRequested()) {
            if (StartLeft) {
                drive.followTrajectorySequence(trajLeft);

                switch (detectedID) {
                    case 1:
                        // Park zone 2 LEFT
                        drive.followTrajectorySequence(parkLeft1);
                        break;

                    case 2:
                        // Park zone 3 MID
                        drive.followTrajectorySequence(parkLeft2);
                        break;

                    default:
                        // Park zone 1 RIGHT
                        drive.followTrajectorySequence(parkLeft3);
                        break;
                }
            } else {
                drive.followTrajectorySequence(trajRight);

                switch (detectedID) {
                    case 1:
                        // Park zone 1
                        //drive.followTrajectorySequence(parkRight1);
                        break;

                    case 2:
                        // Park zone 2
                        //drive.followTrajectorySequence(parkRight2);
                        break;

                    default:
                        // Park zone 3
                        //drive.followTrajectorySequence(parkRight3);
                        break;
                }
            }
        }

        sliderLeft.setTargetPosition(0);
        sliderRight.setTargetPosition(0);

        sliderLeft.setPower(0.25);
        sliderRight.setPower(0.25);

        while (opModeIsActive() && Math.abs(0 - sliderLeft.getCurrentPosition()) >= 4) {
            // Move sliders down
        }

        sliderLeft.setPower(0);
        sliderRight.setPower(0);
    }
}