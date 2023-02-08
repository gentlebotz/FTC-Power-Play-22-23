package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Hardware;


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

@TeleOp(name = "DrivingPP", group = "Iterative Opmode")
//@Disabled
public class DrivingPowerPlay extends OpMode {
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

    private boolean handClosed = true;
    private boolean xPressed;
    private boolean yPressed;

    // Slider, Intake variables
    private int target = 0;
    private int sliderSpeed = 200;
    private int current;
    private int incr;
    private int maxHeight = 6000;
    private int minHeight = -10;
    private boolean sliderLimits = true;
    private boolean limitsPressed = false;

    private ElapsedTime intakeTimer = new ElapsedTime();

    private enum LiftState{
        LIFT_START,
        LIFT_LOW,
        LIFT_MID,
        LIFT_HIGH
    };

    LiftState liftState = LiftState.LIFT_START;

    private int lowPole = 1400;
    private int midPole = 2700;
    private int highPole = 3950;

    private double intakeArmPickupPosition = 0.8;
    private double intakeArmMidPosition = 0.5;
    private double intakeArmDropPosition = 0;

    private double handOpenPos = 0.29;
    private double handClosedPos = 0.1;

    private enum ArmState{
        ARM_INTAKE,
        ARM_MID_TO_DROP,
        ARM_MID_TO_INTAKE,
        ARM_DROP
    }

    ArmState armState = ArmState.ARM_MID_TO_DROP;

    private boolean aPressed = false;

    public static double mapRange(double a1, double a2, double b1, double b2, double s){
        return b1 + ((s - a1)*(b2 - b1))/(a2 - a1);
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status: ", "Busy");
        telemetry.update();

        // Initialize hardware variables
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        sliderLeft = hardwareMap.get(DcMotor.class, "sliderLeft");
        sliderRight = hardwareMap.get(DcMotor.class, "sliderRight");
        sliderLimitSwitch = hardwareMap.get(TouchSensor.class, "sliderLimitSwitch");
        intakeArmServoLeft =  hardwareMap.get(Servo.class, "intakeArmServoL");
        intakeArmServoRight = hardwareMap.get(Servo.class, "intakeArmServoR");
        intakeHand = hardwareMap.get(Servo.class, "intakeHand");

        //  Motor Direction
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        sliderLeft.setDirection(DcMotor.Direction.FORWARD);
        sliderRight.setDirection(DcMotor.Direction.REVERSE);
        intakeArmServoRight.setDirection(Servo.Direction.REVERSE);

        // Set encoder mode
        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sliderLeft.setTargetPosition(0);
        sliderRight.setTargetPosition(0);

        sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeHand.setPosition(handClosedPos);

        intakeTimer.startTime();
        intakeTimer.reset();

        telemetry.addData("Status: ", "Done");
        telemetry.update();
    }

    // Code to run REPEATEDLY after driver hits INIT but before they hit PLAY
    @Override
    public void init_loop() {}

    // Code to be run ONCE after driver hits PLAY
    @Override
    public void start() {
        intakeTimer.reset();
        intakeTimer.startTime();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Gamepad Inputs
        double G1leftStickY = -gamepad1.left_stick_y;
        double G1leftStickX = -gamepad1.left_stick_x;
        double G1rightStickX = gamepad1.right_stick_x;
        double G2rightStickY = -gamepad2.right_stick_y;
        double G2rightStickX = -gamepad2.right_stick_x;
        double G2leftStickY = -gamepad2.left_stick_y;

        // Holonomic mecanum wheel movement
        rightRear.setPower(driveSpeed * (G1leftStickY + -G1leftStickX + 1.2 * -G1rightStickX));
        leftRear.setPower(driveSpeed * (G1leftStickY + G1leftStickX + 1.2 * G1rightStickX));
        rightFront.setPower(driveSpeed * (G1leftStickY + G1leftStickX + 1.2 * -G1rightStickX));
        leftFront.setPower(driveSpeed * (G1leftStickY + -G1leftStickX + 1.2 * G1rightStickX));

        // Update lift target using joystick
        current = sliderLeft.getCurrentPosition();
        incr = (int)(G2leftStickY * sliderSpeed);

        target += incr;

        // Or set lift target using dpad
        if(gamepad2.dpad_up){
            target = highPole;
        }

        if(gamepad2.dpad_left || gamepad2.dpad_right){
            target = midPole;
        }

        if(gamepad2.dpad_down){
            target = lowPole;
        }

        if(gamepad2.left_bumper && !limitsPressed) {
            limitsPressed = true;
            sliderLimits = !sliderLimits;
        } else if (!gamepad2.left_bumper) {
            limitsPressed = false;
        }

        // Limit lift extension & retraction
        if(target > maxHeight && sliderLimits)
        {
            target = maxHeight;
        }

        if(target < minHeight && sliderLimits)
        {
            target = minHeight;
        }

        // Prevent target from going farther than slider speed
        if(Math.abs(target-current) > sliderSpeed)
        {
            target = current + (int)Math.signum(target-current) * sliderSpeed;
        }

        sliderLeft.setTargetPosition(target);
        sliderRight.setTargetPosition(target);

        // Power motors when more than 10 encoder ticks away from target
        if(Math.abs(target - sliderLeft.getCurrentPosition()) >= 10 || Math.abs(target - sliderRight.getCurrentPosition()) >= 10){
            sliderLeft.setPower(0.75);
            sliderRight.setPower(0.75);
        }

        // Disable motors when sliders are at bottom
        else if(sliderLeft.getCurrentPosition() < 10 || sliderRight.getCurrentPosition() < 10){
            sliderRight.setPower(0);
            sliderLeft.setPower(0);
        }

        // Hold motors in position with low power
        else {
            sliderLeft.setPower(0.1);
            sliderRight.setPower(0.1);
        }

        // Reset intake position
        if (gamepad1.y && !yPressed && armState != armState.ARM_DROP) {
            armState = ArmState.ARM_DROP;
            yPressed = true;
        }

        else if(!gamepad2.y) {
            yPressed = false;
        }

        // Set intake position using finite state machine
        if(gamepad2.a && !aPressed) {
            aPressed = true;
            intakeTimer.reset();

            switch (armState) {
                case ARM_DROP:
                    armState = ArmState.ARM_MID_TO_INTAKE; // Next state
                    intakeArmServoLeft.setPosition(intakeArmDropPosition);
                    intakeArmServoRight.setPosition(intakeArmDropPosition);
                    break;

                case ARM_MID_TO_INTAKE:
                    armState = ArmState.ARM_INTAKE; // Next state
                    intakeArmServoLeft.setPosition(intakeArmMidPosition);
                    intakeArmServoRight.setPosition(intakeArmMidPosition);

                    // Close hand when moving over middle bridge
                    intakeHand.setPosition(handClosedPos);
                    handClosed = true;
                    break;

                case ARM_INTAKE:
                    armState = ArmState.ARM_MID_TO_DROP; // Next state
                    intakeArmServoLeft.setPosition(intakeArmPickupPosition);
                    intakeArmServoRight.setPosition(intakeArmPickupPosition);
                    break;

                case ARM_MID_TO_DROP:
                    armState = ArmState.ARM_DROP; // Next state
                    intakeArmServoLeft.setPosition(intakeArmMidPosition);
                    intakeArmServoRight.setPosition(intakeArmMidPosition);

                    // Close hand when moving over middle bridge
                    intakeHand.setPosition(handClosedPos);
                    handClosed = true;
                    break;

                default:
                    armState = ArmState.ARM_DROP;
            }
        }

        else if (!gamepad2.a) {
            aPressed = false;
        }

        // Set hand position
        if(gamepad2.x && !xPressed) {
            xPressed = true;
            handClosed = !handClosed;
            intakeHand.setPosition(handClosed ? handClosedPos : handOpenPos);
        }

        else if(!gamepad2.x) {
            xPressed = false;
        }

        // Driving turbo mode
        if (gamepad1.left_bumper && turboStop) {
            turboStop = false;
            turbo = !turbo;
            driveSpeed = turbo ? turboPower : drivePower;
        }

        else if (!gamepad1.left_bumper) {
            turboStop = true;
        }

        // Driver Hub Telemetry
        telemetry.addData("Power mode: ", turbo ? "Turbo" : "No turbo");
        telemetry.addData("Runtime", runtime);
        telemetry.addData("Slider limits: ", sliderLimits ? "On" : "Off");
        telemetry.addData("Limit Switch", !sliderLimitSwitch.isPressed());
        telemetry.addData("Slider Left: ", sliderLeft.getCurrentPosition());
        telemetry.addData("Slider Right: ", sliderRight.getCurrentPosition());
        telemetry.addData("Slider Target ", target);
        telemetry.addData("Arm State: ", armState);
        telemetry.addData("Arm Hand: ", intakeHand.getPosition());

        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Turn off all motor power
        rightRear.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        sliderLeft.setPower(0);
        sliderRight.setPower(0);
    }

}