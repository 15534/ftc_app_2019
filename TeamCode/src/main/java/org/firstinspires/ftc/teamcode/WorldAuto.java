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

            moveTank(0.6, -4, -4, 3, false);
            strafe(0.4, 2, 2, false);
            moveTank(0.6, 4, 4, 3, false);
            //90 dergee turn
            moveTank(0.6, 10.5, -10.5, 5, false);

            if(position == -1){//left
                strafe(STRAFE_SPEED, -16, 10, false);
                moveTank(0.6, 8, 8, 6, false);
            }if(position == 0){//center
                moveTank(0.6, 4, 4, 5, false);
            }else{//right
                strafe(STRAFE_SPEED, 15, 10, false);
                moveTank(0.6, 8, 8, 6, false);
            }


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

    public boolean strafe(double speed,
                              double inches,
                              double timeoutS, boolean withDetector) {

            int newFrontLeftTarget, newBackLeftTarget, newFrontRightTarget, newBackRightTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newFrontLeftTarget = robot.leftFrontDrive.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.leftBackDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                newFrontRightTarget = robot.rightFrontDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.rightBackDrive.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

                robot.leftFrontDrive.setTargetPosition(newFrontLeftTarget);
                robot.leftBackDrive.setTargetPosition(newBackLeftTarget);
                robot.rightFrontDrive.setTargetPosition(newFrontRightTarget);
                robot.rightBackDrive.setTargetPosition(newBackRightTarget);

                // Turn On RUN_TO_POSITION
                robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.leftBackDrive.setPower(Math.abs(speed) * Math.signum(inches));
                robot.rightBackDrive.setPower(Math.abs(speed) * Math.signum(inches));
                robot.leftFrontDrive.setPower(Math.abs(speed) * Math.signum(inches));
                robot.rightFrontDrive.setPower(Math.abs(speed) * Math.signum(inches));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.rightBackDrive.isBusy() && robot.rightFrontDrive.isBusy()
                                && robot.leftBackDrive.isBusy() && robot.leftFrontDrive.isBusy())
                        && (!withDetector || !detector.getAligned())) {

                    // Display it for the driver.
                    telemetry.addData("Current",  "Running to %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                    telemetry.addData("backRight",  "Running at bl: %7d, fl: %7d, br: %7d, fr: %7d",
                            robot.leftBackDrive.getCurrentPosition(),
                            robot.leftFrontDrive.getCurrentPosition(),
                            robot.rightBackDrive.getCurrentPosition(),
                            robot.rightFrontDrive.getCurrentPosition());

                    telemetry.update();
                }

                // Stop all motion;
                robot.rightFrontDrive.setPower(0);
                robot.leftFrontDrive.setPower(0);
                robot.rightBackDrive.setPower(0);
                robot.leftBackDrive.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(MOVE_DELAY);

                if (detector.getAligned()) return true;

            }
            return false;
        }

        public void moveTank(double speed,
                                 double leftInches, double rightInches,
                                 double timeoutS, boolean withDetector) {

                int newFrontLeftTarget, newBackLeftTarget, newFrontRightTarget, newBackRightTarget;

                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                    // Determine new target position, and pass to motor controller
                    newFrontLeftTarget = robot.leftFrontDrive.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
                    newBackLeftTarget = robot.leftBackDrive.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
                    newFrontRightTarget = robot.rightFrontDrive.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
                    newBackRightTarget = robot.rightBackDrive.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);

                    robot.leftFrontDrive.setTargetPosition(newFrontLeftTarget);
                    robot.leftBackDrive.setTargetPosition(newBackLeftTarget);
                    robot.rightFrontDrive.setTargetPosition(newFrontRightTarget);
                    robot.rightBackDrive.setTargetPosition(newBackRightTarget);

                    // Turn On RUN_TO_POSITION
                    robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // reset the timeout time and start motion.
                    runtime.reset();
                    robot.leftBackDrive.setPower(Math.abs(speed));
                    robot.rightBackDrive.setPower(Math.abs(speed));
                    robot.leftFrontDrive.setPower(Math.abs(speed));
                    robot.rightFrontDrive.setPower(Math.abs(speed));

                    // keep looping while we are still active, and there is time left, and both motors are running.
                    // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                    // its target position, the motion will stop.  This is "safer" in the event that the robot will
                    // always end the motion as soon as possible.
                    // However, if you require that BOTH motors have finished their moves before the robot continues
                    // onto the next step, use (isBusy() || isBusy()) in the loop test.
                    while (opModeIsActive() &&
                            (runtime.seconds() < timeoutS) &&
                            (robot.rightBackDrive.isBusy() && robot.rightFrontDrive.isBusy()
                                    && robot.leftBackDrive.isBusy() && robot.leftFrontDrive.isBusy()) && (!withDetector || !detector.getAligned())) {

                        // Display it for the driver.
                        telemetry.addData("Current",  "Running to %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                        telemetry.addData("backRight",  "Running at bl: %7d, fl: %7d, br: %7d, fr: %7d",
                                robot.leftBackDrive.getCurrentPosition(),
                                robot.leftFrontDrive.getCurrentPosition(),
                                robot.rightBackDrive.getCurrentPosition(),
                                robot.rightFrontDrive.getCurrentPosition());

                        telemetry.update();
                    }

                    // Stop all motion;
                    robot.rightFrontDrive.setPower(0);
                    robot.leftFrontDrive.setPower(0);
                    robot.rightBackDrive.setPower(0);
                    robot.leftBackDrive.setPower(0);

                    // Turn off RUN_TO_POSITION
                    robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    // Delay after movement
                    sleep(MOVE_DELAY);
                }
            }
}
