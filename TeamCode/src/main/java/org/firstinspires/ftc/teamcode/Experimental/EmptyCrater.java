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

@Autonomous(name = "Timed Crater", group = "Coldbot")

public class EmptyCrater extends LinearOpMode{

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
                //encoderDrive(0.5, 12, -12, 4);

                /* Step 4. Square the robot up with the wall */
                //encoderDrive(0.2, -6, -6, 4);

                /* Step 5. Drive Forward to position to sample */

                runtime.reset();
                /* Step 6. Sample the Mineral then return to position */
                switch (sampleLocation) {
                    case "UNKNOWN":
                    case "CENTER":

                        turnRight(0.4, 1000);
                        backward(0.4, 700);
                        forward(0.3, 1650);
//                        forward(.2, 19);
//                        backward(.2, 8);
//                        strafeLeft(.4, 8, 6);
//                        turnLeft(.4, 12.25);
//                        forward(.5, 6);
//                        turnLeft(.4, 6.5);
//                        strafeRight(.4, 16, 5);
//                        forward(.5, 30);
//                        dumpMarker();
//                        backward(.6, 40);
//                        strafeRight(.4, 6, 4);
//                        backward(.2, 2.2);
                        break;

                    case "LEFT":

                        turnRight(0.4, 1000);
                        backward(0.4, 700);
                        forward(0.3, 700);
                        turnLeft(0.3, 800);
                        forward(0.3, 1200);
//                        forward(.2, 10);
//                        strafeLeft(.4, 8.2, 6);
//                        forward(.3, 10);
//                        backward(.3, 10);
//                        turnLeft(.4, 12.25);
//                        forward(.5, 5.5);
//                        turnLeft(.4, 6.5);
//                        strafeRight(.4, 14, 5);
//                        forward(.6, 20);
//                        dumpMarker();
//                        backward(.6, 35);
//                        backward(.2, 5);
//                        strafeRight(.4, 6, 4);
//                        backward(.2, 4);
                        break;

                    case "RIGHT":

                        turnRight(0.4, 1000);
                        backward(0.4, 500);
                        forward(0.3, 700);
                        turnRight(0.3, 800);
                        forward(0.3, 1200);
//                        forward(.2, 10);
//                        strafeRight(.4, 8, 6);
//                        forward(.3, 7.5);
//                        backward(.3, 7.5);
//                        turnLeft(.4, 13.0);
//                        forward(.5, 20);
//                        turnLeft(.4, 6.5);
//                        strafeRight(.4, 14, 5);
//                        forward(.6, 24);
//                        dumpMarker();
//                        backward(.6, 35);
//                        strafeRight(.4, 6, 4);
//                        backward(.2, 5);
                        break;
                }

                /* End */
                //raiseRobot();
                sleep(30000);
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //gyroDrive(0.3, 12, 0.0);
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

    /*
     * lowerRobot raises the robots arm to its hanging position thus lowering the robot if it is hooked
     */
    private void lowerRobot() {
        encoderHang(0.5, -7000, 10);
        sleep(200);
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

    public void forward(double speed, long time){

        robot.frontLeft.setPower(speed);
        robot.frontRight.setPower(-speed);
        robot.backLeft.setPower(speed);
        robot.backRight.setPower(-speed);
        sleep(time);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }

    public void backward(double speed, long time){

        robot.frontLeft.setPower(-speed);
        robot.frontRight.setPower(speed);
        robot.backLeft.setPower(-speed);
        robot.backRight.setPower(speed);
        sleep(time);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }

    public void turnLeft(double speed, long time){

        robot.frontLeft.setPower(-speed);
        robot.frontRight.setPower(-speed);
        robot.backLeft.setPower(-speed);
        robot.backRight.setPower(-speed);
        sleep(time);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

    }

    public void turnRight(double speed, long time){

        robot.frontLeft.setPower(speed);
        robot.frontRight.setPower(speed);
        robot.backLeft.setPower(speed);
        robot.backRight.setPower(speed);
        sleep(time);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

    }

}
