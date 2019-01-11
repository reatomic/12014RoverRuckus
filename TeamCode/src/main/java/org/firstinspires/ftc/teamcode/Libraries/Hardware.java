/* League Hardware */

package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware
{
    /* Public OpMode members. */

    //Motors
    public DcMotor frontLeft, backLeft, frontRight, backRight, hangElevator, intakeElevator, intakeAdjust, intake, ledLights = null;

    //Servos
    public CRServo servoIntake;

    //Sensors

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        //Motor hardware mapping
        frontLeft  = hwMap.get(DcMotor.class, "frontLeft");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontRight  = hwMap.get(DcMotor.class, "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");
        hangElevator = hwMap.get(DcMotor.class, "hangElevator");
        intakeElevator = hwMap.get(DcMotor.class, "intakeElevator");
        intakeAdjust = hwMap.get(DcMotor.class, "intakeAdjust");
        intake = hwMap.get(DcMotor.class, "intake");
        ledLights = hwMap.get(DcMotor.class, "ledLights");

        //Servo hardware mapping
        servoIntake = hwMap.get(CRServo.class, "teamMarker");

        //Sensor hardware mapping

        //Motor set powers
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        hangElevator.setPower(0);
        intakeElevator.setPower(0);
        intakeAdjust.setPower(0);
        intake.setPower(0);
        ledLights.setPower(0);
        //Continuous Servo set powers

        //Run motors without encoder
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeAdjust.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}

