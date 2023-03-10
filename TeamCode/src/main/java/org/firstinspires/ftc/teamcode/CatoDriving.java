package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "CatoDriving", group = "Iterative Opmode")
//@Disabled
public class CatoDriving extends OpMode {
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Back_Right_Motor;
    private DcMotor Back_Left_Motor;
    private DcMotor Front_Right_Motor;
    private DcMotor Front_Left_Motor;

    // ....

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status: ", "Busy");
        telemetry.update();

        // Initialize hardware variables
        Back_Right_Motor = hardwareMap.get(DcMotor.class, "rightRear");
        Back_Left_Motor = hardwareMap.get(DcMotor.class, "leftRear");
        Front_Right_Motor = hardwareMap.get(DcMotor.class, "rightFront");
        Front_Left_Motor = hardwareMap.get(DcMotor.class, "leftFront");

        Back_Left_Motor.setDirection(DcMotor.Direction.REVERSE);
        Front_Left_Motor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status: ", "Done");
        telemetry.update();
    }

    // Code to run REPEATEDLY after driver hits INIT but before they hit PLAY
    @Override
    public void init_loop() {}

    // Code to be run ONCE after driver hits PLAY
    @Override
    public void start() {

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


        Back_Right_Motor.setPower(G1leftStickY);
        Back_Left_Motor.setPower(G1leftStickY);
        Front_Right_Motor.setPower(G1leftStickY);
        Front_Left_Motor.setPower(G1leftStickY);


        telemetry.addData("Runtime", runtime);

        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Turn off all motor power
    }

}