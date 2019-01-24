package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Libraries.Hardware;

import java.util.List;
import java.util.Locale;

@Autonomous(name = "Experimental Depot", group = "Coldbot")

public class ExperimentalDepot extends LinearOpMode
{

    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.05;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.4;     // Larger is more responsive, but also less stable

//    static final double COUNTS_PER_MOTOR_REV = 1120;
//    static final double DRIVE_GEAR_REDUCTION = 0.51;
//    static final double WHEEL_DIAMETER_INCHES = 6;

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 0.51;
    static final double WHEEL_DIAMETER_INCHES = 6;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798214808651328230664709384460955058223172535940812848111745028410270193852110555964462294895493038196442881097566593344612847564823378678316527120190914564856692346034861045432664821339360726024914127372458700660631558817488152092096282925409171536436789259036001133053054882046652138414695194151160943305727036575959195309218611738193261179310511854807446237996274956735188575272489122793818301194912983367336244065664308602139494639522473719070217986094370277053921717629317675238467481846766940513200056812714526356082778577134275778960917363717872146844090122495343014654958537105079227968925892354201995611212902196086403441815981362977477130996051870721134999999837297804995105973173281609631859502445945534690830264252230825334468503526193118817101000313783875288658753320838142061717766914730359825349042875546873115956286388235378759375195778185778053217122680661300192787661119590921642019893809525720106548586327886593615338182796823030195203530185296899577362259941389124972177528347913151557485724245415069595082953311686172785588907509838175463746493931925506040092770167113900984882401285836160356370766010471018194295559619894676783744944825537977472684710404753464620804668425906949129331367702898915210475216205696602405803815019351125338243003558764024749647326391419927260426992279678235478163600934172164121992458631503028618297455570674983850549458858692699569092721079750930295532116534498720275596023648066549911988183479775356636980742654252786255181841757467289097777279);

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AeWbHOX/////AAABmU9GHfAlN0CJgNed4l/4qrseJ0TsGVFEMRaWpMvpOi5s8CW0iiayYB5YkoDgiqFkJexDQsxfRIVpnA+iCCsrYqZXBTIu66lWASvyynGsattVV49V5Bp+BRuxywn0m6pnJRXFlwjnvgHR7xoUrRpE6Pwir0lIlpUIBJREYw9uMc6eTL3yedJstdgV40zwUOwPzwe++1GQ+34JISHpnIZ4xPca+uAtCPje1h3XeR1PP/HHk1/2tNhKz4XVYtYVq5+6ev/8Ca+D9t9j5wXSvi3FOSZmCPVICYO+vWGeEFzeWxmvC34mAPZoZfwGVcz4HYgdRl4tJiIC19VSuW+7iFX/7/GOI/TPFNnnz3EUJOTfFQiy";
    /**
     * #vuforia is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * #tfod is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    private String sampleLocation = "UNKNOWN";
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;
    private Telemetry.Item encoderProjected;
    private Telemetry.Item encoderCurrent;
    private Telemetry.Item hangProjected;
    private Telemetry.Item hangCurrent;



    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "imu";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        robot.hangElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Wait until we're told to go
        waitForStart();
        /* Step 1: Lower Robot */
        lowerRobot();

        if (opModeIsActive()) {
            /* Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
                sleep(2000);
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    sampleLocation = "LEFT";
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    sampleLocation = "RIGHT";
                                } else {
                                    sampleLocation = "CENTER";
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
                /* Begin Auto Path Movement */
                /*  */
                /* Step 3. Turn the robot */
                encoderDrive(0.5, 22, -22, 4);

                /* Step 5. Drive Forward to position to sample */
                encoderDrive(0.5, -8, -8, 4);

                runtime.reset();
                /* Step 6. Sample the Mineral then return to position */
                switch (sampleLocation) {
                    case "UNKNOWN":
                    case "CENTER":
                        //gyroDrive(.25, 55, 0);
                        forward(.25,55 );
                        dumpMarker();
                        backward(.4, 27);
                        turnLeft(.3, 24);
                        forward(0.5, 38);
                        turnLeft(.3, 11);
                        strafeRight(.3, 12, 3);
                        forward(0.4, 18);

                        break;

                    case "LEFT":
                        forward(.2, 14);
                        strafeLeft(.4, 19, 6);
                        forward(.3, 50);
                        turnRight(.3, 11);
                        forward(.3, 5);
                        dumpMarker();
                        backward(0.5, 50);
                        turnLeft(0.5,48);
                        strafeRight(.3, 12, 3);
                        forward(0.3, 15);

                        break;

                    case "RIGHT":
                        forward(.2, 14);
                        strafeRight(.4, 19, 6);
                        forward(.3, 50);
                        turnLeft(.3, 13);
                        forward(.3, 5);
                        dumpMarker();
                        forward(.4, 6);
                        backward(.3, 3);
                        strafeLeft(.3, 4, 3);
                        turnLeft(.3, 15);
                        forward(.3, 16);
                        turnLeft(.3, 11.5);
                        strafeRight(.3, 16, 3);
                        forward(0.5, 45);
                        forward(0.3, 20);
                        break;
                }

                /* End */
                lowerArm();
                raiseRobot();
                sleep(30000);
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

    }

    //Methods
    //_____________________________________________________________________________________________________________________________________

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


    public double getHeading(){
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        return heading;
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() - moveCounts;
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + moveCounts;
            newBackLeftTarget = robot.backLeft.getCurrentPosition() - moveCounts;
            newBackRightTarget = robot.backRight.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.backLeft.setTargetPosition(newBackLeftTarget);
            robot.backRight.setTargetPosition(newBackRightTarget);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.frontRight.setPower(speed);
            robot.frontLeft.setPower(speed);
            robot.backRight.setPower(speed);
            robot.backLeft.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy() && robot.backLeft.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.frontLeft.setPower(leftSpeed);
                robot.frontRight.setPower(rightSpeed);
                robot.backLeft.setPower(leftSpeed);
                robot.backRight.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d: %7d: %7d: %7d ",      newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Actual",  "%7d: %7d: %7d: %7d ",      robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition() );
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (double speed, double angle) {

        double heading = 0;
        char direction = 'n';

        if(angle <= 0)
        {direction = 'r';}
        else if (angle >= 0)
        {direction = 'l';}

        if (opModeIsActive()){
            // keep looping while we are still active, and not on heading.
            while (opModeIsActive() && ((heading <= (angle - 0.5)) || (heading >= (angle + 0.5)))) {
                Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;
                telemetry.addData("Heading", heading);
                telemetry.update();
                // start motion.
                if(direction == 'r') {
                    speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                    robot.frontRight.setPower(-speed);
                    robot.frontLeft.setPower(-speed);
                    robot.backRight.setPower(-speed);
                    robot.backLeft.setPower(-speed);
                }
                if(direction == 'l'){
                    speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                    robot.frontRight.setPower(speed);
                    robot.frontLeft.setPower(speed);
                    robot.backRight.setPower(speed);
                    robot.backLeft.setPower(speed);
                }
            }
            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);


        }
    }
    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.frontLeft.setPower(leftSpeed);
        robot.backLeft.setPower(leftSpeed);
        robot.frontRight.setPower(rightSpeed);
        robot.backRight.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /*
     * raiseRobot lowers the robots arm to it's starting position thus raising the robot in the air if it is hooked
     */
    private void raiseRobot() {
        encoderHang(0.8, 0, 8);
        sleep(200);
    }

    /*
     * lowerRobot raises the robots arm to its hanging position thus lowering the robot if it is hooked
     */
    private void lowerRobot() {
        encoderHang(0.5, -6400, 10);
        sleep(200);
    }

    private void lowerArm() {
        robot.intakeAdjust.setPower(1);
        sleep(800);
        robot.intakeAdjust.setPower(0);
    }

    /*
     * forward moves the robot forward at the prescribed speed for the prescribed distance
     *
     * @param spd
     * @param fwd
     */
    private void forward(double spd, double fwd) {
        encoderDrive(spd, fwd, fwd, 10.0);
        sleep(200);
    }

    /*
     * backward moves the robot backward at the prescribed speed for the prescribed distance
     *
     * @param spd
     * @param back
     */
    private void backward(double spd, double back) {
        encoderDrive(spd, -back, -back, 5.0);
        sleep(200);
    }

    /*
     * turnRight turns the robot right at the prescribed speed for the prescribed distance
     *
     * @param spd
     * @param turn
     */
    private void turnRight(double spd, double turn) {
        encoderDrive(spd, turn, -turn, 5.0);
        sleep(200);
    }

    /*
     * turnLeft turns the robot right at the prescribed speed for the prescribed distance
     *
     * @param spd
     * @param turn
     */
    private void turnLeft(double spd, double turn) {
        encoderDrive(spd, -turn, turn, 5.0);
        sleep(200);
    }
    //
//    public void strafeLeft(double spd, double strafe, double angle){
//        strafeDrive(spd, strafe, angle, 'L');
//
//    }
//
//    public void strafeRight(double spd, double strafe, double angle){
//        strafeDrive(spd, strafe, angle, 'R');
//    }
    public void strafeLeft(double spd, double strafe, double timeoutS){
        strafeDrive(spd, strafe, strafe, -strafe, -strafe, timeoutS);

    }

    public void strafeRight(double spd, double strafe, double timeoutS){
        strafeDrive(spd, -strafe, -strafe, strafe, strafe, timeoutS);
    }


    /*
     * Dump the team marker
     */
    private void dumpMarker() {
        robot.servoIntake.setPower(1);
        sleep(3200);
        robot.servoIntake.setPower(0);
    }

    /*
     * Lowers intake **DEPRECATED**
     */
    private void lowerIntake(){
        robot.intakeElevator.setPower(0.4);
        sleep(2500);
        robot.intakeElevator.setPower(0);
    }

    /*
     * Powers intake for use in crater or team marker
     *
     */
    private void useIntake(){
        robot.servoIntake.setPower(-1);
        sleep(8000);
        robot.servoIntake.setPower(0);
    }

    /*
     * encoderDrive allows the robot to go at a certain speed for a certain distance based
     * on ticks of the encoder. This allows the robot to be very precise in it's movements
     *
     * @param speed
     * @param leftInches
     * @param rightInches
     * @param timeoutS
     */

    private void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {

        int newFrontRightTarget;
        int newFrontLeftTarget;
        int newBackRightTarget;
        int newBackLeftTarget;
        encoderProjected = telemetry.addData("Projected Path", 0);
        encoderCurrent = telemetry.addData("Current Path", 0);


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (-leftInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.backRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int) (-leftInches * COUNTS_PER_INCH);

            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.backRight.setTargetPosition(newBackRightTarget);
            robot.backLeft.setTargetPosition(newBackLeftTarget);
            // Turn On RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontRight.setPower(Math.abs(speed));
            robot.frontLeft.setPower(Math.abs(speed));
            robot.backRight.setPower(Math.abs(speed));
            robot.backLeft.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontRight.isBusy() && robot.frontLeft.isBusy() && robot.backRight.isBusy() && robot.backLeft.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newFrontRightTarget, newFrontLeftTarget,
//                        newBackRightTarget, newBackLeftTarget);

//                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
//                        robot.frontRight.getCurrentPosition(),
//                        robot.frontLeft.getCurrentPosition(),
//                        robot.backRight.getCurrentPosition(),
//                        robot.backLeft.getCurrentPosition());
                encoderProjected.setValue("Running to %7d :%7d :%7d :%7d", newFrontRightTarget, newFrontLeftTarget,
                        newBackRightTarget, newBackLeftTarget);
                encoderCurrent.setValue("Running at %7d :%7d :%7d :%7d",
                        robot.frontRight.getCurrentPosition(),
                        robot.frontLeft.getCurrentPosition(),
                        robot.backRight.getCurrentPosition(),
                        robot.backLeft.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void encoderHang(double speed, int hangTarget, double timeoutS) {


        hangProjected = telemetry.addData("Projected Hang Position", 0);
        hangCurrent = telemetry.addData("Current Hang Position", 0);
        telemetry.update();
        if (opModeIsActive()) {


            robot.hangElevator.setTargetPosition(hangTarget);

            robot.hangElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.hangElevator.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.hangElevator.isBusy()) {

//                telemetry.addData("Path1", "Running to %7d", hangTarget);
//                telemetry.addData("Path2", "Running at %7d", robot.hangElevator.getCurrentPosition());
                hangProjected.setValue("Running to %7d", hangTarget);
                hangCurrent.setValue("Running at %7d", robot.hangElevator.getCurrentPosition());
                telemetry.update();

            }

            robot.hangElevator.setPower(0);

            robot.hangElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public void strafeDrive(double speed, double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches, double timeoutS) {

        int newFrontRightTarget;
        int newFrontLeftTarget;
        int newBackRightTarget;
        int newBackLeftTarget;
        encoderProjected = telemetry.addData("Projected Path", 0);
        encoderCurrent = telemetry.addData("Current Path", 0);


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.backRight.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);

            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.backRight.setTargetPosition(newBackRightTarget);
            robot.backLeft.setTargetPosition(newBackLeftTarget);
            // Turn On RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontRight.setPower(Math.abs(speed));
            robot.frontLeft.setPower(Math.abs(speed));
            robot.backRight.setPower(Math.abs(speed));
            robot.backLeft.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontRight.isBusy() && robot.frontLeft.isBusy() && robot.backRight.isBusy() && robot.backLeft.isBusy())) {

                // Display it for the driver.
                encoderProjected.setValue("Running to %7d :%7d :%7d :%7d", newFrontRightTarget, newFrontLeftTarget,
                        newBackRightTarget, newBackLeftTarget);
                encoderCurrent.setValue("Running at %7d :%7d :%7d :%7d",
                        robot.frontRight.getCurrentPosition(),
                        robot.frontLeft.getCurrentPosition(),
                        robot.backRight.getCurrentPosition(),
                        robot.backLeft.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

//    public void strafeDrive(double speed, double distance, double angle, char direction) {
//
//        int     newFrontLeftTarget = 0;
//        int     newFrontRightTarget = 0;
//        int     newBackLeftTarget = 0;
//        int     newBackRightTarget = 0;
//        int     moveCounts;
//        double  max;
//        double  error;
//        double  steer;
//        double  leftSpeed;
//        double  rightSpeed;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            if(direction == 'L') {
//                // Determine new target position, and pass to motor controller
//                moveCounts = (int) (distance * COUNTS_PER_INCH);
//                newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + moveCounts;
//                newFrontRightTarget = robot.frontRight.getCurrentPosition() + moveCounts;
//                newBackLeftTarget = robot.backLeft.getCurrentPosition() - moveCounts;
//                newBackRightTarget = robot.backRight.getCurrentPosition() - moveCounts;
//            }
//
//            if (direction == 'R'){
//                // Determine new target position, and pass to motor controller
//                moveCounts = (int) (distance * COUNTS_PER_INCH);
//                newFrontLeftTarget = robot.frontLeft.getCurrentPosition() - moveCounts;
//                newFrontRightTarget = robot.frontRight.getCurrentPosition() - moveCounts;
//                newBackLeftTarget = robot.backLeft.getCurrentPosition() + moveCounts;
//                newBackRightTarget = robot.backRight.getCurrentPosition() + moveCounts;
//            }
//
//            // Set Target and Turn On RUN_TO_POSITION
//            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
//            robot.frontRight.setTargetPosition(newFrontRightTarget);
//            robot.backLeft.setTargetPosition(newBackLeftTarget);
//            robot.backRight.setTargetPosition(newBackRightTarget);
//
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // start motion.
//            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
//            robot.frontRight.setPower(speed);
//            robot.frontLeft.setPower(speed);
//            robot.backRight.setPower(speed);
//            robot.backLeft.setPower(speed);
//
//            // keep looping while we are still active, and BOTH motors are running.
//            while (opModeIsActive() &&
//                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backRight.isBusy() && robot.backLeft.isBusy())) {
//
//                // adjust relative speed based on heading error.
//                error = getError(angle);
//                steer = getSteer(error, P_DRIVE_COEFF);
//
//                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0)
//                    steer *= -1.0;
//
//                leftSpeed = speed - steer;
//                rightSpeed = speed + steer;
//
//                // Normalize speeds if either one exceeds +/- 1.0;
//                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//                if (max > 1.0)
//                {
//                    leftSpeed /= max;
//                    rightSpeed /= max;
//                }
//
//                robot.frontLeft.setPower(leftSpeed);
//                robot.frontRight.setPower(rightSpeed);
//                robot.backLeft.setPower(leftSpeed);
//                robot.backRight.setPower(rightSpeed);
//
//                // Display drive status for the driver.
//                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
//                telemetry.addData("Target",  "%7d: %7d: %7d: %7d ",      newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget,  newBackRightTarget);
//                telemetry.addData("Actual",  "%7d: %7d: %7d: %7d ",      robot.frontLeft.getCurrentPosition(),
//                        robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition() );
//                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            robot.frontLeft.setPower(0);
//            robot.frontRight.setPower(0);
//            robot.backLeft.setPower(0);
//            robot.backRight.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }
}
