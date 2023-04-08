package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PIDF test", group = "Iterative Opmode")
//@Disabled
public class PIDFtest extends OpMode {
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Motor1;

    // ....

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status: ", "Busy");
        telemetry.update();

        // Initialize hardware variables
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");





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

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Turn off all motor power
    }

}