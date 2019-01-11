/* Fire Wires TeleOp */

/* Package */
package org.firstinspires.ftc.teamcode.Final;

/* Imports */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Libraries.Hardware;
import org.firstinspires.ftc.teamcode.Libraries.FireBot;

/* TeleOp name & group */
@TeleOp(name = "TeleOp", group = "FireBot")
public class FinalTeleOp extends OpMode {

    /* Declare OpMode members. */
    FireBot firebot = new FireBot();
    Hardware robot = new Hardware();

    /* TeleOp Initialization */
    @Override
    public void init(){

        robot.init(hardwareMap);
        robot.hangElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /* Start loop for TeleOp */
    @Override
    public void start(){

    }

    /* Teleop Loop */
    @Override
    public void loop(){



        //Declaring and initializing gamepad controls

        //Joystick intialization and conditioning
        //original configuration: db: 0 , off: 0.05 , gain: 0.9
        double gamepad1LeftY = firebot.joystick_conditioning(gamepad1.left_stick_y, 0, 0.05, 0.9);
        double gamepad1LeftX = firebot.joystick_conditioning(gamepad1.left_stick_x, 0, 0.05, 0.9);
        double gamepad1RightX = firebot.joystick_conditioning(gamepad1.right_stick_x, 0, 0.05, 0.9);
        double gamepad2LeftY = firebot.joystick_conditioning(gamepad2.left_stick_y, 0, 0.05, 0.9);
        double hangElevatorPower = firebot.gamepad_conditioning(gamepad2.right_stick_y, 0, 0.05, 0.9);

        //Trigger initialization and conditioning
        double gamepad1RightTrigger = firebot.gamepad_conditioning(gamepad1.right_trigger, 0, 0.05, 0.9);
        double gamepad1LeftTrigger = firebot.gamepad_conditioning(gamepad1.left_trigger, 0, 0.05, 0.9);
        double gamepad2RightTrigger = firebot.gamepad_conditioning(gamepad2.right_trigger, 0, 0.05, 0.9);
        double gamepad2LeftTrigger = firebot.gamepad_conditioning(gamepad2.left_trigger, 0, 0.05, 0.9);


        //D-Pad initialization
        boolean dpadUp = gamepad1.dpad_up; //Directional Pad: Up
        boolean dpadDown = gamepad1.dpad_down; //Directional Pad: Down
        boolean dpadUp2 = gamepad2.dpad_up;
        boolean dpadDown2 = gamepad2.dpad_down;


        //Mecanum values
        double maxPower = 0.6; //Maximum power for power range
        double yMove = gamepad1LeftY; //Y-Axis movement
        double xMove = gamepad1LeftX; //X-Axis movement
        double cMove = gamepad1RightX; //Rotating-Axis movement
        double frontLeftPower = 0; //Front Left motor power
        double frontRightPower = 0; //Front Right motor power
        double backLeftPower = 0; //Back Left motor power
        double backRightPower = 0; //Back Right motor power

        //If statement to prevent power from being sent to the same motor from multiple sources
        if (!dpadUp && !dpadDown && gamepad1LeftTrigger == 0 && gamepad1RightTrigger == 0) {
            //Calculating Mecanum power
            frontLeftPower = yMove - xMove - cMove;
            frontRightPower = -yMove - xMove - cMove;
            backLeftPower = yMove + xMove - cMove;
            backRightPower = -yMove + xMove - cMove;


            //Limiting power values from -1 to 1 to conform to setPower() limits
            frontLeftPower = Range.clip(frontLeftPower, -maxPower, maxPower);
            frontRightPower = Range.clip(frontRightPower, -maxPower, maxPower);
            backLeftPower = Range.clip(backLeftPower, -maxPower, maxPower);
            backRightPower = Range.clip(backRightPower, -maxPower, maxPower);

            //Setting power to Mecanum drivetrain
            robot.frontLeft.setPower(frontLeftPower);
            robot.frontRight.setPower(frontRightPower);
            robot.backLeft.setPower(backLeftPower);
            robot.backRight.setPower(backRightPower);
        }

        //Alternative inputs active for drive
        else {
            //Strafe left
            if(gamepad1LeftTrigger > 0){
                robot.frontLeft.setPower(gamepad1LeftTrigger);
                robot.frontRight.setPower(gamepad1LeftTrigger);
                robot.backLeft.setPower(-gamepad1LeftTrigger);
                robot.backRight.setPower(-gamepad1LeftTrigger);
            }

            //Strafe right
             if(gamepad1RightTrigger > 0){

                    robot.frontLeft.setPower(-gamepad1RightTrigger);
                    robot.frontRight.setPower(-gamepad1RightTrigger);
                    robot.backLeft.setPower(gamepad1RightTrigger);
                    robot.backRight.setPower(gamepad1RightTrigger);

            }

            //Full forward power
            if(dpadUp){
                robot.frontLeft.setPower(-1);
                robot.frontRight.setPower(1);
                robot.backLeft.setPower(-1);
                robot.backRight.setPower(1);
            }

            //Full reverse power
            if (dpadDown){
                robot.frontLeft.setPower(1);
                robot.frontRight.setPower(-1);
                robot.backLeft.setPower(1);
                robot.backRight.setPower(-1);
            }
        }

        if (dpadUp2 || dpadDown2){
            if (dpadUp2){ robot.intakeAdjust.setPower(0.8); }
            if (dpadDown2){ robot.intakeAdjust.setPower(-0.8); }
        }

        else { robot.intakeAdjust.setPower(0); }



        //Setting power to manipulators
        double intake = gamepad2RightTrigger;
        double outtake = -gamepad2LeftTrigger;

        robot.intakeElevator.setPower(gamepad2LeftY);
        robot.hangElevator.setPower(hangElevatorPower);

        //Setting power to intake
        //Contingency against loop error
        if (intake > 0){
            robot.servoIntake.setPower(intake);

        }
        else{
            robot.servoIntake.setPower(outtake);

        }

        if(frontRightPower > 0 )
        {
            robot.ledLights.setPower(frontRightPower * .5);
        }
        else { robot.ledLights.setPower(0.2); }



        //Find encoder ticks for hang
        robot.hangElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Current Hang Position", robot.hangElevator.getCurrentPosition());
        telemetry.update();

    }

    public void stop(){

        robot.ledLights.setPower(0);
    }

    /* // If we aren't moving right now check for "brake mode" to hold position
        if ((Math.abs(right) < 0.1) && ((Math.abs(left) < 0.1) && gamepad1.x)) {
            // Requesting brake mode to hold position against defense (e.g. for shooting)
            if (!braked) {
                // First time we see this condition to setup brake mode
                DbgLog.msg("DM10337 -- Setting braked mode.");
                braked = true;
                robot.setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
                // Record where we are at and set it as motor target to hold
                lfBrakedPosn = robot.lfDrive.getCurrentPosition();
                lrBrakedPosn = robot.lrDrive.getCurrentPosition();
                rfBrakedPosn = robot.rfDrive.getCurrentPosition();
                rrBrakedPosn = robot.rrDrive.getCurrentPosition();
                robot.lfDrive.setTargetPosition(lfBrakedPosn);
                robot.lrDrive.setTargetPosition(lrBrakedPosn);
                robot.rfDrive.setTargetPosition(rfBrakedPosn);
                robot.rrDrive.setTargetPosition(rrBrakedPosn);
                robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
                // Allow up to max power to hold our position
                robot.lfDrive.setPower(1.0);
                robot.rfDrive.setPower(1.0);
                robot.rfDrive.setPower(1.0);
                robot.rrDrive.setPower(1.0);
            } else {
                // already in brake mode -- nothing to do but log if we are having to push
                if (robot.lfDrive.isBusy() || robot.lrDrive.isBusy() ||
                        robot.rfDrive.isBusy() || robot.rrDrive.isBusy()) {
                    DbgLog.msg("DM10337 -- Being pushed and fighting back.");
                }
            }
        }


        if (braked && !gamepad1.x) {
            // We are leaving braked mode
            braked = false;
            DbgLog.msg("DM10337 -- Leaving brake mode");
            robot.setDriveZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.lfDrive.setPower(0.0);
            robot.lrDrive.setPower(0.0);
            robot.rfDrive.setPower(0.0);
            robot.rrDrive.setPower(0.0);
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (!braked) {
            // Not braked so we can set the motors to power requested by joysticks
            // And lets drive
            robot.lfDrive.setPower(left);
            robot.lrDrive.setPower(left);
            robot.rfDrive.setPower(right);
            robot.rrDrive.setPower(right);
        }*/
}
