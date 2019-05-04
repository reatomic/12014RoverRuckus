package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Libraries.FireBot;
import org.firstinspires.ftc.teamcode.Libraries.Hardware;

/* TeleOp name & group */
@TeleOp(name = "MotorTester", group = "FireBot")
public class MotorTesting extends OpMode {

    private FireBot firebot = new FireBot();
    public DcMotor test;
    public Hardware hwMap = null;

    public void init(){

        test = hardwareMap.get(DcMotor.class, "test");
        test.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        test.setPower(0);
        test.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void start(){

    }

    public void loop(){

        if(gamepad1.left_stick_y > 0.1) {


        }

        else{

        }

        test.getCurrentPosition();

        telemetry.addData("Joystick 1:LY Output: ", -gamepad1.left_stick_y);
        telemetry.update();


    }

    public void stop(){

    }


}
