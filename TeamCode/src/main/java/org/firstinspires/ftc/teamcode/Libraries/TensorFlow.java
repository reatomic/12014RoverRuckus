/* League TensorFlow */
package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;



@TeleOp(name = "TensorFlow")
@Disabled
public class TensorFlow extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";




    private ElapsedTime runtime = new ElapsedTime();
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;



    @Override
    public void runOpMode() {


        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        parameters.vuforiaLicenseKey = "Abt2+kr/////AAAAGfohNMSr50Hhsmc0NKURDXxcEYe+3v84BQHHk1+GauJvwdKGzq4nAImDbhHuI5gqWVPAh10O2BMlt8WgMwc/bFGrhC/ZY1GH2xUouSTIZeQIRvn81na+3Py372ZU+wRjlCDB0p1G35HhRXkg525FsXJjxPkOvLlu1dTI5gRYFmS5bKxuWFSycgutn0pVIEplVI+lP/Zu1skKXuhSoT6DvxzPfXsbg8PyvzJvJXr2iSitOl1dV3dwwlbCFI5kAjFhFH7dDvIBJnWGqckMhceWjCxhTubE8hg6tXhtur3m4t/77rWlLErBH3RIvs8tDjoH/ke93TRQeEwDTJIPFgoNE5JscMUPbtXhTL3kn3OhI5I2";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        waitForStart();
        resetStartTime();


        tensorDetect();

    }

    String sampleLocation = "UNDETERMINED";



    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public String tensorDetect() {
        telemetry.addData("Tensorflow: ", "Activated");

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            runtime.reset();
            while (sampleLocation == "UNDETERMINED" && getRuntime() < 3) {
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
                    }
                    telemetry.update();
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        return sampleLocation;
    }
}

//    public int tensorDetect() {
//
//        if (opModeIsActive()) {
//            /** Activate Tensor Flow Object Detection. */
//            if (tfod != null) {
//                tfod.activate();
//            }
//
//            while (sampleLocation == 0 && getRuntime() < 2) {
//                if (tfod != null) {
//                    // getUpdatedRecognitions() will return null if no new information is available since
//                    // the last time that call was made.
//                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                    if (updatedRecognitions != null) {
//                        telemetry.addData("# Object Detected", updatedRecognitions.size());
//                        if (updatedRecognitions.size() == 2) {
//                            int goldMineralX = -1;
//                            int silverMineral1X = -1;
//                            //int silverMineral2X = -1;
//                            for (Recognition recognition : updatedRecognitions) {
//                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                                    goldMineralX = (int) recognition.getLeft();
//                                } else if (silverMineral1X == -1) {
//                                    silverMineral1X = (int) recognition.getLeft();
//                                } /*else {
//                                    silverMineral2X = (int) recognition.getLeft();
//                                }*/
//                            }
//                            if (goldMineralX != -1 && silverMineral1X != -1 /*silverMineral2X != -1*/) {
//                                if (goldMineralX < silverMineral1X) {
//                                    telemetry.addData("Gold Mineral Position", "Left");
//                                    sampleLocation = 1;
//                                } else if (goldMineralX > silverMineral1X) {
//                                    telemetry.addData("Gold Mineral Position", "Center");
//                                    sampleLocation = 2;
//                                } else {
//                                    telemetry.addData("Gold Mineral Position", "Right");
//                                    sampleLocation = 3;
//                                }
//                            }
//                        }
//                        /*else {
//                         hit center
//                        }*/
//                        telemetry.addData("sample int value", sampleLocation);
//                        telemetry.update();
//                    }
//                }
//            }
//        }
//
//        if (tfod != null) {
//            tfod.shutdown();
//        }
//        return sampleLocation;
//    }
//}