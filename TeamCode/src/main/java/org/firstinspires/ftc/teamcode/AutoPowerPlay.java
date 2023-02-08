package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

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

    ;

    LiftState liftState = LiftState.LIFT_START;

    private int lowPole = 1400;
    private int midPole = 2700;
    private int highPole = 3950;

    private double intakeArmPickupPosition = 0.8;
    private double intakeArmMidPosition = 0.5;
    private double intakeArmDropPosition = 0;

    private double handOpenPos = .29;
    private double handClosedPos = 0.15;

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
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution

    @Override
    public void runOpMode() throws InterruptedException {
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

        //Encoders
        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /*
        Initialize EasyOpenCV
         */

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //OpenCV Pipeline
        powerplayPipeline myPipeline;
        webcam.setPipeline(myPipeline = new powerplayPipeline()); // Was PipeLine

        // Configuration of Pipeline
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        /*
        Initialize RoadRunner
         */

        Pose2d startPoseLeft = new Pose2d(-36, -60, Math.toRadians(90));
        Pose2d startPoseRight = new Pose2d(36, -60, Math.toRadians(90));

        /*
        Left side trajectories
         */
        TrajectorySequence trajLeft = drive.trajectorySequenceBuilder(startPoseLeft)
                // Drop preloaded cone
                // Sliders up
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                    sliderRight.setPower(0.4);
                                    sliderLeft.setPower(0.4);

                                    sliderRight.setTargetPosition(highPole);
                                    sliderLeft.setTargetPosition(highPole);
                })

                .lineToConstantHeading(new Vector2d(-18.6, -58.33)) // Move to F3
                .splineToConstantHeading(new Vector2d(-13, -40), Math.toRadians(90)) // Move to E3

                // Slider up, prepare outtake
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    intakeArmServoLeft.setPosition(0.2);
                                    intakeArmServoRight.setPosition(0.2);
                })

                .waitSeconds(.4)

                .lineToLinearHeading(new Pose2d(-4, -27.33, Math.toRadians(45))) // 45 deg hi approach

                // Drop cone
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                    intakeHand.setPosition(handOpenPos);
                })

                .waitSeconds(3)

                // Sliders down
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                                    sliderRight.setPower(0.4);
                                    sliderLeft.setPower(0.4);

                                    sliderRight.setTargetPosition(lowPole);
                                    sliderLeft.setTargetPosition(lowPole);
                })

                .lineToLinearHeading(new Pose2d(-11.67, -35, Math.toRadians(90))) // Reverse approach back to E3
                .lineToConstantHeading(new Vector2d(-11.67, -12.2)) // Move to D3 and rotate for cycles

                // Cycles
                // .lineToLinearHeading(new Pose2d(-35, -11.67, Math.toRadians(-180))) // Move to B3 and rotate for cycles
                .turn(Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    intakeArmServoLeft.setPosition(intakeArmPickupPosition);
                                    intakeArmServoRight.setPosition(intakeArmPickupPosition);
                })

                .lineToConstantHeading(new Vector2d(-58.33, -12.2)) // Move to cone stack D1

                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                                    sliderRight.setPower(0.2);
                                    sliderLeft.setPower(0.2);

                                    sliderRight.setTargetPosition(lowPole - 100);
                                    sliderLeft.setTargetPosition(lowPole - 100);
                })

                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
                                    intakeHand.setPosition(handClosedPos);
                                    sliderRight.setTargetPosition(lowPole + 200);
                                  sliderLeft.setTargetPosition(lowPole + 200);
                })

                .waitSeconds(2)

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                                    sliderRight.setPower(0.4);
                                    sliderLeft.setPower(0.4);

                                    sliderRight.setTargetPosition(highPole);
                                    sliderLeft.setTargetPosition(highPole);
                })

                .lineToConstantHeading(new Vector2d(-35, -12.2)) // Move to D2
                .lineToLinearHeading(new Pose2d(-29.33, -6, Math.toRadians(225))) // 45 deg hi approach

                // Drop cone
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                    intakeHand.setPosition(handOpenPos);
                })

                .waitSeconds(2)

                // Sliders down
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                                    sliderRight.setPower(0.4);
                                    sliderLeft.setPower(0.4);

                                    sliderRight.setTargetPosition(lowPole);
                                    sliderLeft.setTargetPosition(lowPole);
                })

                .lineToLinearHeading(new Pose2d(-35, -12.2, Math.toRadians(-180))) // Reverse approach back to D2

                // Sliders down and setup for driver-controlled
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    sliderRight.setPower(0.4);
                                    sliderLeft.setPower(0.4);

                                    sliderRight.setTargetPosition(0);
                                    sliderLeft.setTargetPosition(0);

                                    intakeArmServoLeft.setPosition(intakeArmPickupPosition);
                                    intakeArmServoRight.setPosition(intakeArmPickupPosition);
                                    intakeHand.setPosition(handOpenPos);
                })

                // Parking
                .lineToConstantHeading(new Vector2d(-58.33, -12.2)) // Location 1
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-35, -12.2)) // Location 2
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-11.67, -12.2)) // Location 3

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
                .lineToConstantHeading(new Vector2d(-58.33, -11.67)) // Location 1
                .build();

        TrajectorySequence parkLeft2 = drive.trajectorySequenceBuilder(trajLeft.end())
                .waitSeconds(1) // Location 2
                .build();

        TrajectorySequence parkLeft3 = drive.trajectorySequenceBuilder(trajLeft.end())
                .lineToConstantHeading(new Vector2d(-11.67, -11.67)) // Location 3
                .build();

        TrajectorySequence parkRight3 = drive.trajectorySequenceBuilder(trajRight.end())
                .lineToConstantHeading(new Vector2d(58.33, -11.67)) // Location 3
                .build();

        TrajectorySequence parkRight2 = drive.trajectorySequenceBuilder(trajRight.end())
                .waitSeconds(1) // Location 2
                .build();

        TrajectorySequence parkRight1 = drive.trajectorySequenceBuilder(trajRight.end())
                .waitSeconds(2)
                .lineToConstantHeading(new Vector2d(11.67, -11.67)) // Location 1
                .build();

        drive.setPoseEstimate(startPoseLeft);

        // Telemetry
        telemetry.addData("Status: ", "Busy");
        telemetry.update();

        // Pickup preloaded cone
        intakeHand.setPosition(handClosedPos);
        sleep(700);
        intakeArmServoLeft.setPosition(intakeArmMidPosition);
        intakeArmServoRight.setPosition(intakeArmMidPosition);
        sleep(1000);

        // Slider calibration
        sliderRight.setTargetPosition(400);
        sliderLeft.setTargetPosition(400);

        sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setPower(0.5);
        sliderLeft.setPower(0.5);
        while (Math.abs(400 - sliderLeft.getCurrentPosition()) >= 10 && opModeIsActive()) {
            //Do nothing
        }
        sliderRight.setPower(0);
        sliderLeft.setPower(0);

        // Move slider down until it reaches limit switch
        sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!sliderLimitSwitch.isPressed() && sliderRight.getCurrentPosition() >= -400 && opModeIsActive()) {
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

        //intakeArmServo.setPosition(intakeArmMidPosition);
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        while (!isStarted()) {
            if (gamepad1.dpad_left && !StartLeft) {
//                telemetry.addData("Starting location: ", "Left Side");
                StartLeft = true;
                drive.setPoseEstimate(startPoseLeft);
            } else if (gamepad1.dpad_right && StartLeft) {
//                telemetry.addData("Starting location: ", "Right Side");
                StartLeft = false;
                drive.setPoseEstimate(startPoseRight);
            }

            telemetry.addData("Starting location: ", StartLeft ? "Left side" : "Right side");
            telemetry.addData("Parking location: ", myPipeline.getLocation());
            telemetry.update();
        }

        //Wait until start button is pressed
        waitForStart();

        while (opModeIsActive()) {
            if (myPipeline.error) {
                telemetry.addData("Exception: ", myPipeline.debug);
            }

            telemetry.addData("Park location: ", myPipeline.getLocation());
            telemetry.update();

            if (!isStopRequested()) {
                if (StartLeft) {
                    drive.followTrajectorySequence(trajLeft);

                    switch (myPipeline.getLocation()) {
                        case MID:
                            // Park zone 2 MID
                            drive.followTrajectorySequence(parkLeft1);
                            break;

                        case RIGHT:
                            // Park zone 3 RIGHT
                            drive.followTrajectorySequence(parkLeft2);
                            break;

                        default:
                            // Park zone 1 LEFT
                            drive.followTrajectorySequence(parkLeft3);
                            break;
                    }
                } else {
                    drive.followTrajectorySequence(trajRight);

                    switch (myPipeline.getLocation()) {
                        case MID:
                            // Park zone 1
                            //drive.followTrajectorySequence(parkRight1);
                            break;

                        case RIGHT:
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

        }
        sliderLeft.setTargetPosition(0);
        sliderRight.setTargetPosition(0);
        sliderLeft.setPower(0.25);
        sliderRight.setPower(0.25);
        while (opModeIsActive() && Math.abs(0 - sliderLeft.getCurrentPosition()) >= 4) {
        }
        sliderLeft.setPower(0);
        sliderRight.setPower(0);
    }
}