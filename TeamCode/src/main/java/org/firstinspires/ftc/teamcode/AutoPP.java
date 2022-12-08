//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.message.redux.StopOpMode;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.sun.tools.javac.tree.JCTree;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.opencv.core.Scalar;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.Vector;
//
//@Autonomous (name = "AutoPP", group = "AutoRR")
//public class AutoPP extends LinearOpMode {
//
//    /*
//    Robot Vars
//     */
//    // Wheels
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor rightRear = null;
//    private DcMotor leftRear = null;
//    private DcMotor rightFront = null;
//    private DcMotor leftFront = null;
//
//
//    // Slider, Intake
//    private DcMotor sliderLeft = null;
//    private DcMotor sliderRight = null;
//    private TouchSensor sliderLimitSwitch = null;
//    private Servo intakeArmServo = null;
//    private CRServo intakeWheelServo = null;
//
//    // Variables
//    private double power = 1;
//    private double power2 = .4;
//    private boolean turboStop = true;
//    private boolean turbo = false;
//
//    private int target = 0;
//    private int speed2 = 120;
//    private int current;
//    private int incr;
//    private int maxHeight = 3000;
//    private int minHeight = -10;
//
//    private enum LiftState{
//        LIFT_START,
//        LIFT_LOW,
//        LIFT_MID,
//        LIFT_HIGH
//    };
//
//    LiftState liftState = LiftState.LIFT_START;
//
//    private int lowPole = 0;
//    private int midPole = 0;
//    private int highPole = 0;
//
//    private double intakeArmPickupPosition = 0;
//    private double intakeArmMidPostition = 0.5;
//    private double intakeArmDropPosition = 1;
//
//    private enum ArmState{
//        ARM_INTAKE,
//        ARM_MID,
//        ARM_DROP
//    }
//
//    ArmState armState = ArmState.ARM_INTAKE;
//
//
//    public static double mapRange(double a1, double a2, double b1, double b2, double s){
//        return b1 + ((s - a1)*(b2 - b1))/(a2 - a1);
//    }
//
//    private boolean ParkLocation = true;
//    private boolean StartLocation = false;
//    private boolean ParkRight = true;
//
//    /*
//    OpenCV Vars
//    */
//    private OpenCvCamera webcam;
//
//    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
//    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
//
//    double CrLowerUpdate = 150;
//    double CbLowerUpdate = 120;
//    double CrUpperUpdate = 255;
//    double CbUpperUpdate = 255;
//
//    double lowerruntime = 0;
//    double upperruntime = 0;
//
//    // Black Range                                     Y   Cr   Cb
//    public static Scalar scalarLowerYCrCb = new Scalar(0, 123, 124);
//    public static Scalar scalarUpperYCrCb = new Scalar(70, 135, 135);
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
////        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
////        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
////        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
////        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
//        sliderLeft = hardwareMap.get(DcMotor.class, "sliderLeft");
//        sliderRight = hardwareMap.get(DcMotor.class, "sliderRight");
//        sliderLimitSwitch = hardwareMap.get(TouchSensor.class, "sliderLimitSwitch");
//        intakeArmServo =  hardwareMap.get(Servo.class, "intakeArmServo");
//        intakeWheelServo = hardwareMap.get(CRServo.class, "intakeWheelServo");
//
//        //  Motor Direction
//        rightRear.setDirection(DcMotor.Direction.FORWARD);
//        leftRear.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.setDirection(DcMotor.Direction.FORWARD);
//        leftFront.setDirection(DcMotor.Direction.REVERSE);
//        sliderLeft.setDirection(DcMotor.Direction.FORWARD);
//        sliderRight.setDirection(DcMotor.Direction.REVERSE);
//
//        //Encoders
//        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        /*
//        Initialize EasyOpenCV
//         */
//
//        // OpenCV webcam
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        //OpenCV Pipeline
//        PipeLine myPipeline;
//        webcam.setPipeline(myPipeline = new PipeLine());
//        // Configuration of Pipeline
//        myPipeline.ConfigurePipeline(30, 30,30,30,  CAMERA_WIDTH, CAMERA_HEIGHT);
//        myPipeline.ConfigureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
//        myPipeline.ConfigureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
//
//        /*
//        Initialize RoadRunner
//         */
//
//        Pose2d startPose = new Pose2d(-34, -60, Math.toRadians(90));
//        Pose2d startPoseLeft = new Pose2d(12,-60, Math.toRadians(90));
//
////        if(StartLocation){
////
////        }
////        else{
////
////        }
//
//        /*
//        Trajectories right side
//         */
//        //Create trajectory to carrousel
//        TrajectorySequence trajToCarrousel = drive.trajectorySequenceBuilder(startPose)
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    motorArm.setTargetPosition(350);
//                    motorArm.setPower(.8);
//                    linearServo.setPosition(mapRange(3800, -100, .12, .50, 350));
//                    armServo.setPosition(0.05);
//                })
//                //.forward(1)
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    motorArm.setPower(0);
//                })
//
//                //Go to carousel
//                .lineToConstantHeading(new Vector2d(-57, -55))
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, () ->
//                        motorWheel.setPower(-.35))
//                .UNSTABLE_addTemporalMarkerOffset(2.5, () ->
//                        motorWheel.setPower(-.2))
//                .waitSeconds(5)
//                .UNSTABLE_addTemporalMarkerOffset(.1, () ->
//                        motorWheel.setPower(0))
//                .build();
//        /*
//        Right Side Trajectories
//         */
//        TrajectorySequence trajSeqLow = drive.trajectorySequenceBuilder(trajToCarrousel.end())
//
//                //Move to wobble tower
//                .lineToSplineHeading(new Pose2d(-60, -24, Math.toRadians(0)))
//                .lineToConstantHeading(new Vector2d(-30, -24))
//
//                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
//                    motorArm.setTargetPosition(targetLow);
//                    linearServo.setPosition(mapRange(3300, -100, .12, .50, targetLow));
//                    motorArm.setPower(.8);
//                })
//
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//                    armServo.setPosition(.24);
//                    motorArm.setPower(0);
//                })
//                .waitSeconds(2)
//                .build();
//
//        TrajectorySequence trajSeqMid = drive.trajectorySequenceBuilder(trajToCarrousel.end())
//
//                //Move to wobble tower
//                .lineToSplineHeading(new Pose2d(-60, -24, Math.toRadians(0)))
//                .lineToConstantHeading(new Vector2d(-30, -24))
//
//                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
//                    motorArm.setTargetPosition(targetMid);
//                    linearServo.setPosition(.2);
//                    motorArm.setPower(.8);
//                })
//
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//                    armServo.setPosition(.24);
//                    motorArm.setPower(0);
//                })
//                .waitSeconds(2)
//                .build();
//
//        TrajectorySequence trajSeqHigh = drive.trajectorySequenceBuilder(trajToCarrousel.end())
//
//                //Move to wobble tower
//                .lineToSplineHeading(new Pose2d(-60, -24, Math.toRadians(0)))
//                .lineToConstantHeading(new Vector2d(-29.5, -24))
//
//                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
//                    motorArm.setTargetPosition(targetHigh);
//                    linearServo.setPosition(mapRange(3300, -100, .12, .50, targetHigh));
//                    motorArm.setPower(.8);
//                })
//
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//                    armServo.setPosition(.24);
//                    motorArm.setPower(0);
//                })
//                .waitSeconds(2)
//                .build();
//
//        /*
//        Left side trajectories
//         */
//        TrajectorySequence trajSeqLowL = drive.trajectorySequenceBuilder(startPoseLeft)
//                //Move to wobble tower
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    motorArm.setTargetPosition(targetLow);
//                    linearServo.setPosition(mapRange(3300, -100, .12, .50, targetLow));
//                    motorArm.setPower(.8);
//                })
//                .waitSeconds(3)
//                .strafeLeft(5)
//                .splineToConstantHeading(new Vector2d(-13, -40), Math.toRadians(90))
//
//
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//                    armServo.setPosition(.24);
//                    motorArm.setPower(0);
//                })
//                .waitSeconds(2)
////                .lineTo(new Vector2d(-6, -50))
//                .lineToLinearHeading(new Pose2d(0, -60, Math.toRadians(0)))
//                .build();
//
//        TrajectorySequence trajSeqMidL = drive.trajectorySequenceBuilder(startPoseLeft)
//
//                //Move to wobble tower
//                .waitSeconds(2)
//                .splineToConstantHeading(new Vector2d(-12.5, -40), Math.toRadians(90))
//
//                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
//                    motorArm.setTargetPosition(targetMid);
//                    linearServo.setPosition(.2);
//                    motorArm.setPower(.8);
//                })
//
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//                    armServo.setPosition(.24);
//                    motorArm.setPower(0);
//                })
//                .waitSeconds(2)
//                .lineToLinearHeading(new Pose2d(0, -60, Math.toRadians(0)))
//                .build();
//
//        TrajectorySequence trajSeqHighL = drive.trajectorySequenceBuilder(startPoseLeft)
//
//                //Move to wobble tower
//                .waitSeconds(2)
//                .splineToConstantHeading(new Vector2d(-12.2, -40), Math.toRadians(90))
//
//                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
//                    motorArm.setTargetPosition(targetHigh);
//                    linearServo.setPosition(mapRange(3300, -100, .12, .50, targetHigh));
//                    motorArm.setPower(.8);
//                })
//
//
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//                    armServo.setPosition(.24);
//                    motorArm.setPower(0);
//                })
//                .waitSeconds(2)
//                .lineToLinearHeading(new Pose2d(0, -60, Math.toRadians(0)))
//                .build();
//
//        TrajectorySequence trajToDepot = drive.trajectorySequenceBuilder(trajSeqLow.end())
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                .lineToConstantHeading(new Vector2d(-60, -24))
//                .lineToConstantHeading(new Vector2d(-60, -36))
//                //.resetVelConstraint()
//                .build();
//
//        telemetry.addData("Status: ", "Initialized");
//        telemetry.update();
//
//        drive.setPoseEstimate(startPose);
//        while(!isStarted()){
//            if(gamepad1.dpad_right && !StartLocation){
//                telemetry.addData("Starting location: ", "Left Side");
//                StartLocation = true;
//                drive.setPoseEstimate(startPoseLeft);
//            }
//            if(gamepad1.dpad_left && StartLocation){
//                telemetry.addData("Starting location: ", "Right Side");
//                StartLocation = false;
//                drive.setPoseEstimate(startPose);
//            }
//
//            if(gamepad1.x && !ParkLocation){
//                telemetry.addData("Parking location: ", "Warehouse");
//                ParkLocation = true;
//            }
//            if(gamepad1.b && ParkLocation){
//                telemetry.addData("Parking location: ", "Depot");
//                ParkLocation = false;
//            }
//
//
//            if(gamepad1.y && ParkRight){
//                telemetry.addData("WarehousePark location: ", "Left Side");
//                ParkRight = false;
//            }
//            if(gamepad1.a && !ParkRight){
//                telemetry.addData("WarehousePark location: ", "Right Side");
//                ParkRight = true;
//            }
//
//            telemetry.addData("Starting location: ", StartLocation ? "Warehouse" : "Carousel");
//            telemetry.addData("Parking location: ", ParkLocation ? "Warehouse" : "Depot");
//            telemetry.addData("WarehouseParking location: ", ParkRight ? "Right" : "Left");
//            telemetry.update();
//        }
//
//        TrajectorySequence trajToWarehouse = drive.trajectorySequenceBuilder(StartLocation ? trajSeqLowL.end() : trajToDepot.end())
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                //.lineToConstantHeading(new Vector2d(-50, 25))
//                //.splineToConstantHeading(new Vector2d(-58, 30), Math.toRadians(90))
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    linearServo.setPosition(mapRange(3300, -100, .12, .50, 0));
//                    motorArm.setTargetPosition(1000);
//                    motorArm.setPower(.8);
//                    armServo.setPosition(0.05);
//                })
//
//                .UNSTABLE_addTemporalMarkerOffset(2.75,() -> {
//                    motorArm.setTargetPosition(0);
//                })
//
//                .UNSTABLE_addTemporalMarkerOffset(3,() -> {
//                    motorArm.setPower(0);
//                })
//                //.resetVelConstraint()
//                //.splineToConstantHeading(new Vector2d(5, 66), Math.toRadians(0))
//                .lineToLinearHeading(new Pose2d(15, -64, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(45, -67), Math.toRadians(0))
//                .build();
//
//        TrajectorySequence trajToWarehouseCarousel = drive.trajectorySequenceBuilder(trajToDepot.end())
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(32.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                //.lineToConstantHeading(new Vector2d(-50, 25))
//                //.splineToConstantHeading(new Vector2d(-58, 30), Math.toRadians(90))
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    linearServo.setPosition(mapRange(3300, -100, .12, .50, 0));
//                    motorArm.setTargetPosition(1000);
//                    motorArm.setPower(.8);
//                    armServo.setPosition(0.05);
//                })
//
//                .UNSTABLE_addTemporalMarkerOffset(2.75,() -> {
//                    motorArm.setTargetPosition(0);
//                })
//
//                .UNSTABLE_addTemporalMarkerOffset(3,() -> {
//                    motorArm.setPower(0);
//                })
//                //.resetVelConstraint()
//                //.splineToConstantHeading(new Vector2d(5, 66), Math.toRadians(0))
//
//                .splineToConstantHeading(new Vector2d(-50, -60), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(45, -67), Math.toRadians(0))
//                .build();
//
//        TrajectorySequence trajParkRight = drive.trajectorySequenceBuilder(trajToWarehouse.end())
//                .strafeLeft(30)
//                .build();
//
//        //Wait until start button is pressed
//        //waitForStart();
//
//        while (opModeIsActive()) {
//            if (myPipeline.error) {
//                telemetry.addData("Exception: ", myPipeline.debug);
//            }
//
//            telemetry.addData("RectArea: ", myPipeline.getRectArea());
//            telemetry.update();
//
////            if (myPipeline.getRectArea() > 1000) {
//            if (myPipeline.getRectMidpointX() > 400) {
//                //AUTONOMOUS_C();
//                webcam.stopStreaming();
//                telemetry.addData("Marker location: ", "Right");
//                telemetry.update();
//                if (!isStopRequested()) {
//                    if(StartLocation){
//                        drive.followTrajectorySequence(trajSeqHighL);
//                        drive.followTrajectorySequence(trajToWarehouse);
//                        if(ParkRight) {
//                            drive.followTrajectorySequence(trajParkRight);
//                        }
//                    }
//                    if(!StartLocation) {
//                        drive.followTrajectorySequence(trajToCarrousel);
//                        drive.followTrajectorySequence(trajSeqHigh);
//                        drive.followTrajectorySequence(trajToDepot);
//                        if (ParkLocation) {
//                            sleep(5000);
//                            drive.followTrajectorySequence(trajToWarehouseCarousel);
//                        }
//                    }
//                }
//            } else if (myPipeline.getRectMidpointX() > 200) {
//                //AUTONOMOUS_B();
//                webcam.stopStreaming();
//                telemetry.addData("Marker location: ", "Mid");
//                telemetry.update();
//                if (!isStopRequested()) {
//                    //Starting on left side
//                    if(StartLocation){
//                        drive.followTrajectorySequence(trajSeqMidL);
//                        drive.followTrajectorySequence(trajToWarehouse);
//                        if(ParkRight) {
//                            drive.followTrajectorySequence(trajParkRight);
//                        }
//                    }
//
//                    //Starting on right side
//                    if(!StartLocation){
//                        drive.followTrajectorySequence(trajToCarrousel);
//                        drive.followTrajectorySequence(trajSeqMid);
//                        drive.followTrajectorySequence(trajToDepot);
//                        if(ParkLocation) {
//                            sleep(5000);
//                            drive.followTrajectorySequence(trajToWarehouseCarousel);
//                        }
//                    }
//                }
//            } else {
//                //AUTONOMOUS_A();
//                webcam.stopStreaming();
//                telemetry.addData("Marker location: ", "Left");
//                telemetry.update();
//                if (!isStopRequested()) {
//                    if(StartLocation){
//                        drive.followTrajectorySequence(trajSeqLowL);
//                        drive.followTrajectorySequence(trajToWarehouse);
//                        if(ParkRight) {
//                            drive.followTrajectorySequence(trajParkRight);
//                        }
//                    }
//                    if(!StartLocation) {
//                        drive.followTrajectorySequence(trajToCarrousel);
//                        drive.followTrajectorySequence(trajSeqLow);
//                        drive.followTrajectorySequence(trajToDepot);
//                        if (ParkLocation) {
//                            sleep(5000);
//                            drive.followTrajectorySequence(trajToWarehouseCarousel);
//                        }
//                    }
//                }
//            }
////            }
//            motorArm.setTargetPosition(0);
//            motorArm.setPower(.8);
//            linearServo.setPosition(mapRange(3300, -100, .1, .50, 0));
//            armServo.setPosition(0.05);
//            while (opModeIsActive() && motorArm.isBusy() && linearServo.getPosition() > .12) {
//            }
//            sleep(100);
//            motorArm.setPower(0);
//            break;
//        }
//    }
//}