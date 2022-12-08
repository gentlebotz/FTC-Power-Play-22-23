package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

@TeleOp(name = "Slider test", group = "Iterative Opmode")
//@Disabled
public class SliderTest extends OpMode {
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

    private enum LiftState{
        LIFT_START,
        LIFT_LOW,
        LIFT_MID,
        LIFT_HIGH
    };

    LiftState liftState = LiftState.LIFT_START;

    private int lowPole = 0;
    private int midPole = 0;
    private int highPole = 0;

    private double intakeArmPickupPosition = 0;
    private double intakeArmMidPostition = 0.5;
    private double intakeArmDropPosition = 1;

    private enum ArmState{
        ARM_INTAKE,
        ARM_MID,
        ARM_DROP
    }

    private boolean aPressed = false;

    ArmState armState = ArmState.ARM_INTAKE;

    public static double mapRange(double a1, double a2, double b1, double b2, double s){
        return b1 + ((s - a1)*(b2 - b1))/(a2 - a1);
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override

    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        sliderLeft = hardwareMap.get(DcMotor.class, "sliderLeft");
        sliderRight = hardwareMap.get(DcMotor.class, "sliderRight");
        sliderLimitSwitch = hardwareMap.get(TouchSensor.class, "sliderLimitSwitch");
        intakeArmServo =  hardwareMap.get(Servo.class, "intakeArmServo");
        intakeWheelServo = hardwareMap.get(CRServo.class, "intakeWheelServo");

        //  Motor Direction
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        sliderLeft.setDirection(DcMotor.Direction.FORWARD);
        sliderRight.setDirection(DcMotor.Direction.REVERSE);

        //Encoders
        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status: ", "Busy");
        telemetry.update();

        // Slider calibration
        sliderRight.setTargetPosition(400);
        sliderLeft.setTargetPosition(400);

        sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setPower(0.5);
        sliderLeft.setPower(0.5);
        while(Math.abs(400 - sliderLeft.getCurrentPosition()) >= 10){
            //Do nothing
        }

        // Move slider down until it reaches limit switch
        sliderRight.setPower(0.2);
        sliderLeft.setPower(0.2);

        sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(!sliderLimitSwitch.isPressed()){
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

        telemetry.addData("Status: ", "Done");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {}

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double G1leftStickY = -gamepad1.left_stick_y;
        double G1leftStickX = -gamepad1.left_stick_x;
        double G1rightStickX = gamepad1.right_stick_x;
        double G2rightStickY = -gamepad2.right_stick_y;
        double G2rightStickX = -gamepad2.right_stick_x;
        double G2leftStickY = -gamepad2.left_stick_y;

        rightRear.setPower(power2 * (G1leftStickY + -G1leftStickX + 1.2 * -G1rightStickX));
        leftRear.setPower(power2 * (G1leftStickY + G1leftStickX + 1.2 * G1rightStickX));
        rightFront.setPower(power2 * (G1leftStickY + G1leftStickX + 1.2 * -G1rightStickX));
        leftFront.setPower(power2 * (G1leftStickY + -G1leftStickX + 1.2 * G1rightStickX));

        // Update lift target
        current =  sliderLeft.getCurrentPosition();

        int current = sliderLeft.getCurrentPosition();
        int incr = (int)(-gamepad2.left_stick_y * speed2);

        target += incr;

        // Limit lift extension
        if(target > maxHeight)
        {
            target = maxHeight;
        }

        if(target < minHeight)
        {
            target = minHeight;
        }

        if(Math.abs(target-current) > speed2)
        {
            target = current + (int)Math.signum(target-current)*speed2;
        }

        sliderLeft.setTargetPosition(target);
        sliderRight.setTargetPosition(target);

        // Power motors when more than 10 ticks away from target
        if(Math.abs(target - sliderLeft.getCurrentPosition()) >= 10 || Math.abs(target - sliderRight.getCurrentPosition()) >= 10){
            sliderLeft.setPower(0.4);
            sliderRight.setPower(0.4);
        }

        else{
            sliderLeft.setPower(0);
            sliderRight.setPower(0);
        }

        // Auto set lift height
        if(gamepad2.dpad_up){
            target = highPole;
        }

        if(gamepad2.dpad_left || gamepad2.dpad_right){
            target = midPole;
        }

        if(gamepad2.dpad_down){
            target = lowPole;
        }

        // Auto set intake positions
        switch(armState){
            case ARM_DROP:
                if(gamepad2.a && !aPressed){
                    intakeArmServo.setPosition(intakeArmDropPosition);
                    armState = ArmState.ARM_INTAKE;
                    aPressed = true;
                }
                break;
            case ARM_INTAKE:
                if(gamepad2.a && !aPressed){
                    intakeArmServo.setPosition(intakeArmPickupPosition);
                    armState = ArmState.ARM_MID;
                    aPressed = true;
                }
                break;
            case ARM_MID:
                if(gamepad2.a && !aPressed){
                    intakeArmServo.setPosition(intakeArmMidPostition);
                    armState = ArmState.ARM_DROP;
                    aPressed = true;
                }
                break;
            default:
                armState = ArmState.ARM_DROP;
        }

        if (!gamepad2.a) {
            aPressed = false;
        }

        // Reset intake position
        if (gamepad1.x && armState != armState.ARM_DROP) {
            armState = ArmState.ARM_DROP;
        }

        // Intake wheels control
        if(gamepad2.b){
            intakeWheelServo.setPower(1);
        } else if(gamepad2.y){
            intakeWheelServo.setPower(-1);
        } else {
            intakeWheelServo.setPower(0);
        }

        // Turbo mode
        if (gamepad1.left_bumper && turboStop) {
            turboStop = false;
            turbo = !turbo;
            power2 = turbo ? 1 : 0.5;
        }

        else if (!gamepad1.left_bumper) {
            turboStop = true;
        }

        intakeArmServo.setPosition(G2rightStickX);

        // Telemetry
        telemetry.addData("Power mode: ", turbo ? "Turbo" : "No turbo");
        telemetry.addData("runtime", runtime);
        telemetry.addData("Limit Switch", !sliderLimitSwitch.isPressed());
        telemetry.addData("Slider Left: ", sliderLeft.getCurrentPosition());
        telemetry.addData("Slider Right: ", sliderRight.getCurrentPosition());
        telemetry.addData("Arm State: ", armState);
        telemetry.addData("Arm intake servo: ", G2rightStickX);

        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        rightRear.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        sliderLeft.setPower(0);
        sliderRight.setPower(0);
    }

}