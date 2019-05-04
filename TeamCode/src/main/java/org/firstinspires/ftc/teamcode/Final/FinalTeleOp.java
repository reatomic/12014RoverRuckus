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
    private Hardware robot = new Hardware();
    private FireBot firebot = new FireBot();


    /* TeleOp Initialization */
    @Override
    public void init(){

        robot.init(hardwareMap);
        robot.hangElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        double gamepad2LeftY = firebot.joystick_conditioning(gamepad2.left_stick_y, 0, 0.05, 0.95);
        double intakeAdjustPower = firebot.gamepad_conditioning(gamepad2.right_stick_y, 0, 0.05, 0.95);

        //Trigger initialization and conditioning
        double gamepad1RightTrigger = gamepad1.right_trigger;
        double gamepad1LeftTrigger = gamepad1.left_trigger;
        double gamepad2RightTrigger = firebot.gamepad_conditioning(gamepad2.right_trigger, 0, 0.05, 0.9);
        double gamepad2LeftTrigger = firebot.gamepad_conditioning(gamepad2.left_trigger, 0, 0.05, 0.9);


        //D-Pad initialization
        boolean dpadUp = gamepad1.dpad_up; //Directional Pad: Up
        boolean dpadDown = gamepad1.dpad_down; //Directional Pad: Down
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;
        boolean dpadUp2 = gamepad2.dpad_up;
        boolean dpadDown2 = gamepad2.dpad_down;

        boolean rightBumper2 = gamepad2.right_bumper;
        boolean leftBumper2 = gamepad2.left_bumper;
        rightBumper2 = false;

        //Mecanum values
        double maxPower = .82; //Maximum power for power range
        double yMove = firebot.joystick_conditioning(gamepad1.left_stick_y, 0, 0.05, 0.9);
        double xMove = firebot.joystick_conditioning(gamepad1.left_stick_x, 0, 0.05, 0.9);
        double cMove = firebot.joystick_conditioning(gamepad1.right_stick_x, 0, 0.05, 0.9);
        double frontLeftPower; //Front Left motor power
        double frontRightPower = 0; //Front Right motor power
        double backLeftPower; //Back Left motor power
        double backRightPower = 0; //Back Right motor power

        //If statement to prevent power from being sent to the same motor from multiple sources
        if (!dpadUp && !dpadDown) {
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
            robot.frontLeft.setPower(-frontLeftPower);
            robot.frontRight.setPower(-frontRightPower);
            robot.backLeft.setPower(-backLeftPower);
            robot.backRight.setPower(-backRightPower);
        }

        /*((x - 1.5)^3 + 1.5(x - 1.5)^2) + 0.5) D(0 , 1.5] Max/n = 1.5 */
        //Alternative inputs active for drive
        else {
            //Strafe left
//            if(gamepad1LeftTrigger > 0){
//                robot.frontLeft.setPower(gamepad1LeftTrigger);
//                robot.frontRight.setPower(gamepad1LeftTrigger);
//                robot.backLeft.setPower(-gamepad1LeftTrigger);
//                robot.backRight.setPower(-gamepad1LeftTrigger);
//            }
//
//            //Strafe right
//             if(gamepad1RightTrigger > 0){
//
//                    robot.frontLeft.setPower(-gamepad1RightTrigger);
//                    robot.frontRight.setPower(-gamepad1RightTrigger);
//                    robot.backLeft.setPower(gamepad1RightTrigger);
//                    robot.backRight.setPower(gamepad1RightTrigger);
//
//            }

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

            if(dpadLeft){
                robot.frontLeft.setPower(1);
                robot.frontRight.setPower(1);
                robot.backLeft.setPower(-1);
                robot.backRight.setPower(-1);
            }

            if(dpadRight) {
                robot.frontLeft.setPower(-1);
                robot.frontRight.setPower(-1);
                robot.backLeft.setPower(1);
                robot.backRight.setPower(1);
            }
        }

        if (gamepad1RightTrigger > 0){
            robot.hangElevator.setPower(-gamepad1RightTrigger);

        }
        else{
            robot.hangElevator.setPower(gamepad1LeftTrigger);

        }

        if(gamepad1.a)
        {
            robot.frontRight.setPower(0.4);
        }

        if(gamepad1.b)
        {
            robot.frontLeft.setPower(0.4);
        }

        if(gamepad1.y)
        {
            robot.backRight.setPower(0.4);
        }

        if(gamepad1.x)
        {
            robot.backLeft.setPower(0.4);
        }



        //Setting power to manipulators
        // intake = gamepad2RightTrigger;
        // outtake = -gamepad2LeftTrigger;

        robot.intakeElevator.setPower(-gamepad2LeftY * 0.7);
        robot.intakeAdjust.setPower(intakeAdjustPower * 0.5);
//        if(rightBumper2){
//
//            robot.intakeAdjust.setPower(Math.pow((robot.intakeAdjust.getCurrentPosition() - 1.5) , 3) + 1.5 * Math.pow((robot.intakeAdjust.getCurrentPosition() - 1.5) , 2) + 0.5);
//
//        }
//        else{robot.intakeAdjust.setPower(intakeAdjustPower * 0.5);}
//


        //Setting power to intake
        //Contingency against loop error
        if (gamepad2RightTrigger > 0){
            robot.servoIntake.setPower(gamepad2RightTrigger);

        }
        else{
            robot.servoIntake.setPower(-gamepad2LeftTrigger);

        }




        //Find encoder ticks for hang
        robot.hangElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Current Hang Position", robot.hangElevator.getCurrentPosition());
        telemetry.addData("Front Right", robot.frontRight.getCurrentPosition());
        telemetry.addData("Front Left", robot.frontLeft.getCurrentPosition());
        telemetry.addData("Back Right", robot.backRight.getCurrentPosition());
        telemetry.addData("Back Left", robot.backLeft.getCurrentPosition());
        telemetry.update();

    }

    public void stop(){


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
