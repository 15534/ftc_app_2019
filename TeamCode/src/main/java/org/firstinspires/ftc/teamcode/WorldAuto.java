package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;
@Autonomous(name = "WorldAuto", group = "World Bot")
public class WorldAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public HardwareWorldsRobot robot = new HardwareWorldsRobot();
    double front;
    double back;
    double bottom;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        front = robot.rightfrontsensor.getDistance(DistanceUnit.INCH);
        back = robot.rightbacksensor.getDistance(DistanceUnit.INCH);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Init Successful");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

            robot.phone.setPosition(0);
            sleep(1000);

            /** Activate Tensor Flow Object Detection. */
            int position = -1;

            runtime.reset();

            // detect mineral position

            while (opModeIsActive() && runtime.seconds() < 2) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        int goldMineralX = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            }
                        }

                        if (goldMineralX > 830 && goldMineralX < 1300) position = 1;
                        if (goldMineralX < 830 && goldMineralX > 300) position = 0;
                    }
            }

            if (tfod != null) {
                tfod.shutdown();
            }

            if (position == -1) {
                telemetry.addData("mineral", "left");
            }
            else if (position == 0) {
                telemetry.addData("mineral", "center");
            }
            else {
                telemetry.addData("mineral", "right");
            }
            telemetry.update();


            double bottom = 10;
             robot.actuator.setPower(1);
            while (opModeIsActive() && bottom > 1.86) {
              bottom = bottom();
             telemetry.addData("bottom", bottom);
            }
            sleep(100);
            robot.actuator.setPower(0);
                             sleep(100);
                                        robot.actuator.setPower(0);

                                        drive(0.6);
                                        sleep(200);
                                        drive(0);

                                        strafe(0.4);
                                        sleep(100);
                                        strafe(0);

                                        drive(-0.6);
                                        sleep(200);
                                        drive(0);


        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void drive(double power){
    robot.leftfront.setPower(power);
    robot.rightfront.setPower(power);
    robot.leftback.setPower(power);
    robot.rightback.setPower(power);
    }
        private void strafe(double power) {
            robot.leftfront.setPower(power);
            robot.rightback.setPower(power);
            robot.rightfront.setPower(power);
            robot.leftback.setPower(power);
        }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VertexSettings.VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

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

    private double bottom() {
        double bottom = robot.bottomsensor.getDistance(DistanceUnit.INCH);
        return bottom;
    }

    private int goldMineralX() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
//            telemetry.addData("# Object Detected", updatedRecognitions.size());
            int goldMineralX = -1;
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    goldMineralX = (int) recognition.getLeft();
                }
            }
            return goldMineralX;
        }
        return -2;
    }
}
