package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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
import org.opencv.core.Mat;
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
    private int sliderSpeed = 300;
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

    private int lowPole = 2400;
    private int midPole = 3000;
    private int highPole = 5900;
    private int stackHeight = 1200;

    private double intakeArmDropPosition = 0;
    private double initpickup = 0.15;
    private double intakeArmMidPosition = 0.3;
    private double intakeArmFrontDropPosition = 0.55;
    private double intakeArmPickupPosition = 0.85;

    private double handOpenPos = 0.01;
    private double handClosedPos = 0.13;

    private boolean initHandClosed = false;
    private boolean initHandPressed = false;

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
        sliderLeft.setDirection(DcMotor.Direction.REVERSE);
        sliderRight.setDirection(DcMotor.Direction.FORWARD);
        intakeArmServoRight.setDirection(Servo.Direction.REVERSE);

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

        Pose2d startPoseLeft = new Pose2d(-33, -61.25, Math.toRadians(270));
        Pose2d startPoseRight = new Pose2d(33, -61.25, Math.toRadians(270));

        /*
        Left side trajectories
         */
        TrajectorySequence trajLeft = drive.trajectorySequenceBuilder(startPoseLeft)
                // Drop preloaded cone
                // Sliders up
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        sliderRight.setPower(1);
                        sliderLeft.setPower(1);

                        sliderRight.setTargetPosition(highPole);
                        sliderLeft.setTargetPosition(highPole);
                })

                .lineToConstantHeading(new Vector2d(-33.5, -59))
                .splineToConstantHeading(new Vector2d(-34, -47), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-35, -6), Math.toRadians(90)) // Move to F3
                .lineToConstantHeading(new Vector2d(-35, -13)) // Move to F3

                // Slider up, prepare outtake
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        intakeArmServoLeft.setPosition(intakeArmDropPosition);
                        intakeArmServoRight.setPosition(intakeArmDropPosition);
                })

                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(-26, -5.7, Math.toRadians(260))) // Move to E3

                // Drop cone
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        intakeHand.setPosition(handOpenPos);
                })

                .waitSeconds(.3)

                // Sliders down
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                        sliderRight.setPower(1);
                        sliderLeft.setPower(1);

                        sliderRight.setTargetPosition(lowPole);
                        sliderLeft.setTargetPosition(lowPole);

                        intakeArmServoLeft.setPosition(intakeArmPickupPosition);
                        intakeArmServoRight.setPosition(intakeArmPickupPosition);
                        intakeHand.setPosition(handClosedPos);
                })

                .waitSeconds(.5)

                .build();

        TrajectorySequence leftCycle = drive.trajectorySequenceBuilder(trajLeft.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    sliderLeft.setTargetPosition(lowPole);
                    sliderRight.setTargetPosition(lowPole);
                })

                .lineToLinearHeading(new Pose2d(-35, -13, Math.toRadians(180))) // Move to D3 and rotate for cycles
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(-62,-12.25))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        intakeHand.setPosition(handOpenPos);
                        sliderRight.setTargetPosition(stackHeight);
                        sliderLeft.setTargetPosition(stackHeight);
                })

                // Cycles
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                        intakeHand.setPosition(handClosedPos);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                        sliderRight.setPower(.8);
                        sliderLeft.setPower(.8);
                        sliderRight.setTargetPosition(highPole);
                        sliderLeft.setTargetPosition(highPole);
                })

                .waitSeconds(1.5)

                .lineToConstantHeading(new Vector2d(-40,-14))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intakeArmServoLeft.setPosition(intakeArmDropPosition);
                    intakeArmServoRight.setPosition(intakeArmDropPosition);
                })

                .waitSeconds(.25)

                .lineToLinearHeading(new Pose2d(-31 , -3.5, Math.toRadians(225)))

                // Drop cone
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        intakeHand.setPosition(handOpenPos);
                })

                .waitSeconds(.8)

                // Sliders down
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        sliderRight.setPower(1);
                        sliderLeft.setPower(1);

                        sliderRight.setTargetPosition(lowPole);
                        sliderLeft.setTargetPosition(lowPole);

                        intakeArmServoLeft.setPosition(intakeArmPickupPosition);
                        intakeArmServoRight.setPosition(intakeArmPickupPosition);
                        intakeHand.setPosition(handClosedPos);
                })

//                .lineToLinearHeading(new Pose2d(-35, -13, Math.toRadians(180))) // Move to D3 and rotate for cycles

                .build();

        /*
        Right Side Trajectories
         */

        TrajectorySequence trajRight = drive.trajectorySequenceBuilder(startPoseRight)
                // Drop preloaded cone
                // Sliders up
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    sliderRight.setPower(1);
                    sliderLeft.setPower(1);

                    sliderRight.setTargetPosition(highPole);
                    sliderLeft.setTargetPosition(highPole);
                })

                .lineToConstantHeading(new Vector2d(33.5, -59))
                .splineToConstantHeading(new Vector2d(34, -47), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -6), Math.toRadians(90)) // Move to F3
                .lineToConstantHeading(new Vector2d(35, -13)) // Move to F3

                // Slider up, prepare outtake
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intakeArmServoLeft.setPosition(intakeArmDropPosition);
                    intakeArmServoRight.setPosition(intakeArmDropPosition);
                })

                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(26, -5.7, Math.toRadians(280))) // Move to E3

                // Drop cone
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intakeHand.setPosition(handOpenPos);
                })

                .waitSeconds(.3)

                // Sliders down
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    sliderRight.setPower(1);
                    sliderLeft.setPower(1);

                    sliderRight.setTargetPosition(lowPole);
                    sliderLeft.setTargetPosition(lowPole);

                    intakeArmServoLeft.setPosition(intakeArmPickupPosition);
                    intakeArmServoRight.setPosition(intakeArmPickupPosition);
                    intakeHand.setPosition(handClosedPos);
                })

                .waitSeconds(.5)

                .build();

        TrajectorySequence rightCycle = drive.trajectorySequenceBuilder(trajRight.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    sliderLeft.setTargetPosition(lowPole);
                    sliderRight.setTargetPosition(lowPole);
                })

                .lineToLinearHeading(new Pose2d(35, -13, Math.toRadians(0))) // Move to D3 and rotate for cycles
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(62,-13))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intakeHand.setPosition(handOpenPos);
                    sliderRight.setTargetPosition(stackHeight);
                    sliderLeft.setTargetPosition(stackHeight);
                })

                // Cycles
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    intakeHand.setPosition(handClosedPos);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    sliderRight.setPower(.8);
                    sliderLeft.setPower(.8);
                    sliderRight.setTargetPosition(highPole);
                    sliderLeft.setTargetPosition(highPole);
                })

                .waitSeconds(1.5)

                .lineToConstantHeading(new Vector2d(40,-14))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intakeArmServoLeft.setPosition(intakeArmDropPosition);
                    intakeArmServoRight.setPosition(intakeArmDropPosition);
                })

                .lineToLinearHeading(new Pose2d(29.3, -4, Math.toRadians(300)))

                // Drop cone
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    intakeHand.setPosition(handOpenPos);
                })

                .waitSeconds(.8)

                // Sliders down
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    sliderRight.setPower(1);
                    sliderLeft.setPower(1);

                    sliderRight.setTargetPosition(lowPole);
                    sliderLeft.setTargetPosition(lowPole);

                    intakeArmServoLeft.setPosition(intakeArmPickupPosition);
                    intakeArmServoRight.setPosition(intakeArmPickupPosition);
                    intakeHand.setPosition(handClosedPos);
                })

//                .lineToLinearHeading(new Pose2d(35, -13, Math.toRadians(180))) // Move to D3 and rotate for cycles

                .build();

        /*
        Park trajectories
         */

        TrajectorySequence parkLeft1 = drive.trajectorySequenceBuilder(leftCycle.end())
                .lineToLinearHeading(new Pose2d(-35, -13, Math.toRadians(270))) // Move to D3

                // Sliders down
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    sliderRight.setPower(1);
                    sliderLeft.setPower(1);

                    sliderRight.setTargetPosition(0);
                    sliderLeft.setTargetPosition(0);

                    intakeArmServoLeft.setPosition(intakeArmPickupPosition);
                    intakeArmServoRight.setPosition(intakeArmPickupPosition);
                    intakeHand.setPosition(handClosedPos);
                })


                .lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(270))) // Location 1

                .waitSeconds(.5)
                .build();

        TrajectorySequence parkLeft2 = drive.trajectorySequenceBuilder(leftCycle.end())
                .lineToLinearHeading(new Pose2d(-35, -13, Math.toRadians(270))) // Move to D3

                // Sliders down
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    sliderRight.setPower(1);
                    sliderLeft.setPower(1);

                    sliderRight.setTargetPosition(0);
                    sliderLeft.setTargetPosition(0);

                    intakeArmServoLeft.setPosition(intakeArmPickupPosition);
                    intakeArmServoRight.setPosition(intakeArmPickupPosition);
                    intakeHand.setPosition(handClosedPos);
                })

                .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(270))) // Location 2

                .waitSeconds(.5) // Location 2
                .build();

        TrajectorySequence parkLeft3 = drive.trajectorySequenceBuilder(leftCycle.end())
                .lineToLinearHeading(new Pose2d(-35, -13, Math.toRadians(270))) // Move to D3

                // Sliders down
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    sliderRight.setPower(1);
                    sliderLeft.setPower(1);

                    sliderRight.setTargetPosition(0);
                    sliderLeft.setTargetPosition(0);

                    intakeArmServoLeft.setPosition(intakeArmPickupPosition);
                    intakeArmServoRight.setPosition(intakeArmPickupPosition);
                    intakeHand.setPosition(handClosedPos);
                })

                .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(270))) // Location 3

                .waitSeconds(.5)
                .build();

        TrajectorySequence parkRight3 = drive.trajectorySequenceBuilder(rightCycle.end())
                .lineToLinearHeading(new Pose2d(35, -14, Math.toRadians(270))) // Move to D3

                // Sliders down
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    sliderRight.setPower(1);
                    sliderLeft.setPower(1);

                    sliderRight.setTargetPosition(0);
                    sliderLeft.setTargetPosition(0);

                    intakeArmServoLeft.setPosition(intakeArmPickupPosition);
                    intakeArmServoRight.setPosition(intakeArmPickupPosition);
                    intakeHand.setPosition(handClosedPos);
                })


                .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(270))) // Location 1

                .waitSeconds(.5)
                .build();

        TrajectorySequence parkRight2 = drive.trajectorySequenceBuilder(rightCycle.end())
                .lineToLinearHeading(new Pose2d(35, -14, Math.toRadians(270))) // Move to D3

                // Sliders down
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    sliderRight.setPower(1);
                    sliderLeft.setPower(1);

                    sliderRight.setTargetPosition(0);
                    sliderLeft.setTargetPosition(0);

                    intakeArmServoLeft.setPosition(intakeArmPickupPosition);
                    intakeArmServoRight.setPosition(intakeArmPickupPosition);
                    intakeHand.setPosition(handClosedPos);
                })

                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(270))) // Location 2

                .waitSeconds(.5) // Location 2
                .build();

        TrajectorySequence parkRight1 = drive.trajectorySequenceBuilder(rightCycle.end())
                .lineToLinearHeading(new Pose2d(35, -14, Math.toRadians(270))) // Move to D3

                // Sliders down
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    sliderRight.setPower(1);
                    sliderLeft.setPower(1);

                    sliderRight.setTargetPosition(0);
                    sliderLeft.setTargetPosition(0);

                    intakeArmServoLeft.setPosition(intakeArmPickupPosition);
                    intakeArmServoRight.setPosition(intakeArmPickupPosition);
                    intakeHand.setPosition(handClosedPos);
                })

                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(270))) // Location 3

                .waitSeconds(.5)
                .build();

        drive.setPoseEstimate(startPoseLeft);

        // Pickup preloaded cone
//        intakeHand.setPosition(handClosedPos);
//        sleep(700);
//        intakeArmServoLeft.setPosition(intakeArmMidPosition);
//        intakeArmServoRight.setPosition(intakeArmMidPosition);
//        sleep(1000);

        intakeArmServoRight.setPosition(initpickup);
        intakeArmServoLeft.setPosition(initpickup);

        // Slider calibration
        sliderRight.setTargetPosition(700);
        sliderLeft.setTargetPosition(700);

        sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setPower(0.5);
        sliderLeft.setPower(0.5);

        while (Math.abs(700 - sliderLeft.getCurrentPosition()) >= 10 && !isStopRequested()) {
            // Move slider up to start calibration
        }

        sliderRight.setPower(0);
        sliderLeft.setPower(0);

        // Move slider down until it reaches limit switch
        sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!sliderLimitSwitch.isPressed() && sliderRight.getCurrentPosition() >= -400 && !isStopRequested()) {
            sliderLeft.setPower(-0.25);
            sliderRight.setPower(-0.25);
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
                intakeArmServoLeft.setPosition(initpickup);
                intakeArmServoRight.setPosition(initpickup);
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

        camera.stopStreaming();

        int cycleAmount = 2;
        if (!isStopRequested()) {
            if (StartLeft) {
                drive.followTrajectorySequence(trajLeft);
                for (int i = 0; i < cycleAmount; i++) {
                    drive.followTrajectorySequence(leftCycle);
                }

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
                for (int i = 0; i < cycleAmount; i++) {
                    drive.followTrajectorySequence(leftCycle);
                }

                switch (detectedID) {
                    case 1:
                        // Park zone 1
                        drive.followTrajectorySequence(parkRight1);
                        break;

                    case 2:
                        // Park zone 2
                        drive.followTrajectorySequence(parkRight2);
                        break;

                    default:
                        // Park zone 3
                        drive.followTrajectorySequence(parkRight3);
                        break;
                }
            }
        }

        sliderLeft.setTargetPosition(0);
        sliderRight.setTargetPosition(0);

        sliderLeft.setPower(0.8);
        sliderRight.setPower(0.8);

        while (opModeIsActive() && Math.abs(0 - sliderLeft.getCurrentPosition()) >= 4) {
            // Move sliders down
        }

        sliderLeft.setPower(0);
        sliderRight.setPower(0);
    }
}