package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static android.os.SystemClock.currentThreadTimeMillis;
import static android.os.SystemClock.setCurrentTimeMillis;
import static android.os.SystemClock.sleep;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Forward Auto", group = "AutoRR")
//@Disabled
public class ForwardAuto extends LinearOpMode {
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();

    // Wheels
    private DcMotor rightRear = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;

    // Slider, Intake
    private DcMotor sliderLeft = null;
    private DcMotor sliderRight = null;
    private TouchSensor sliderLimitSwitch = null;
    private Servo intakeArmServo = null;
    private CRServo intakeWheelServo = null;

    // Variables
    private double power = 1;
    private double power2 = .4;
    private boolean turboStop = true;
    private boolean turbo = false;

    private int target = 0;
    private int speed2 = 120;
    private int current;
    private int incr;
    private int maxHeight = 2300;
    private int minHeight = -10;

    private enum LiftState {
        LIFT_START,
        LIFT_LOW,
        LIFT_MID,
        LIFT_HIGH
    }

    LiftState liftState = LiftState.LIFT_START;

    private int lowPole = 0;
    private int midPole = 0;
    private int highPole = 0;

    private double intakeArmPickupPosition = 0.2;
    private double intakeArmMidPosition = 0.5;
    private double intakeArmDropPosition = 0.8;

    private enum ArmState {
        ARM_INTAKE,
        ARM_MID,
        ARM_DROP
    }

    private boolean aPressed = false;

    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution

    ArmState armState = ArmState.ARM_INTAKE;

    public static double mapRange(double a1, double a2, double b1, double b2, double s) {
        return b1 + ((s - a1) * (b2 - b1)) / (a2 - a1);
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {
        // Initialize hardware variables (names must correspond to robot configuration on robot controller app)
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        sliderLeft = hardwareMap.get(DcMotor.class, "sliderLeft");
        sliderRight = hardwareMap.get(DcMotor.class, "sliderRight");
        sliderLimitSwitch = hardwareMap.get(TouchSensor.class, "sliderLimitSwitch");
        //intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServo");
        //intakeWheelServo = hardwareMap.get(CRServo.class, "intakeWheelServo");

        // Motor Direction
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        sliderLeft.setDirection(DcMotor.Direction.FORWARD);
        sliderRight.setDirection(DcMotor.Direction.REVERSE);

        // Set encoder mode
        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        rightRear.setTargetPosition(0);
//        leftRear.setTargetPosition(0);
//        rightFront.setTargetPosition(0);
//        leftFront.setTargetPosition(0);
//
//        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status: ", "Busy");
        telemetry.update();

        // Slider calibration
        sliderRight.setTargetPosition(400);
        sliderLeft.setTargetPosition(400);

        sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setPower(0.5);
        sliderLeft.setPower(0.5);
        while (Math.abs(400 - sliderLeft.getCurrentPosition()) >= 10) {
            // Keep moving up until reaching target
        }

        sliderRight.setPower(0);
        sliderLeft.setPower(0);

        // Move slider down until it reaches limit switch
        sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!sliderLimitSwitch.isPressed() && sliderLeft.getCurrentPosition() >= -400) {
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

        /*
        Initialize EasyOpenCV
         */

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OpenCV Pipeline
        powerplayPipeline myPipeline;
        webcam.setPipeline(myPipeline = new powerplayPipeline()); // Was PipeLine

        // Configuration of Pipeline
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                // Ignore error and hope it works
            }
        });

        telemetry.addData("Status: ", "Done");
        telemetry.update();

        waitForStart();

        if (myPipeline.error) {
            telemetry.addData("Exception: ", myPipeline.debug);
        }

        switch(myPipeline.getLocation()) {
            case MID:
                // Park zone 2 MID
                rightRear.setPower(0.4);
                leftRear.setPower(0.4);
                rightFront.setPower(0.4);
                leftFront.setPower(0.4);

<<<<<<< Updated upstream
                while(rightRear.isBusy() || leftFront.isBusy()){
                    // Do nothing
                }
=======
               sleep(2000);
>>>>>>> Stashed changes

                break;
            case RIGHT:
                // Park zone 3 RIGHT
                rightRear.setPower(0.4);
                leftRear.setPower(-0.4);
                rightFront.setPower(-0.4);
                leftFront.setPower(0.4);

<<<<<<< Updated upstream
                while(rightRear.isBusy() || leftFront.isBusy()){
                    // Do nothing
                }

                rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
=======
               sleep(2000);
>>>>>>> Stashed changes

                rightRear.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                leftFront.setPower(0);

                sleep(500);

                rightRear.setPower(0.4);
                leftRear.setPower(0.4);
                rightFront.setPower(0.4);
                leftFront.setPower(0.4);

<<<<<<< Updated upstream
                while(rightRear.isBusy() || leftFront.isBusy()){
                    // Do nothing
                }
=======
                sleep(2000);
>>>>>>> Stashed changes
                break;
            case LEFT:
                // Park zone 3 RIGHT
                rightRear.setPower(-0.4);
                leftRear.setPower(0.4);
                rightFront.setPower(0.4);
<<<<<<< Updated upstream
                leftFront.setPower(0.4);

                while(rightRear.isBusy() || leftFront.isBusy()){
                    // Do nothing
                }

                rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
=======
                leftFront.setPower(-0.4);
>>>>>>> Stashed changes

                sleep(2000);

                rightRear.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                leftFront.setPower(0);

                sleep(500);

                rightRear.setPower(0.4);
                leftRear.setPower(0.4);
                rightFront.setPower(0.4);
                leftFront.setPower(0.4);

<<<<<<< Updated upstream
                while(rightRear.isBusy() || leftFront.isBusy()){
                    // Do nothing
                }
=======
                sleep(2000);
                break;
            default:
                // Park zone 2 MID
                rightRear.setPower(0.4);
                leftRear.setPower(0.4);
                rightFront.setPower(0.4);
                leftFront.setPower(0.4);

                sleep(2000);

>>>>>>> Stashed changes
                break;
        }

        rightRear.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        sliderLeft.setPower(0);
        sliderRight.setPower(0);
    }

}