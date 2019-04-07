package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
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


    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     STRAFE_SPEED            = 0.5;


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


        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        /** Wait for the game to begin */
        telemetry.addData(">", "Init Successful");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
//


            /*

            moveTank(STRAFE_SPEED, 8, 8, 10);

            // extend intake
            robot.intakeextension.setPower(-1);
            sleep(2000);
            robot.intakeextension.setPower(0);

            // drop intake
            robot.intakeliftleft.setPosition(1.0);
            robot.intakeliftright.setPosition(0.0);

            // release marker
            robot.intake.setPower(0.8);
            sleep(1000);
            robot.intake.setPower(0);

            // raise intake
            robot.intakeliftleft.setPosition(0.0);
            robot.intakeliftright.setPosition(1.0);

            // retract intake
            robot.intakeextension.setPower(1);
            sleep(2000);
            robot.intakeextension.setPower(0);

            moveTank(STRAFE_SPEED, -8, -8, 10);

            */

            robot.phone.setPosition(0);
            sleep(1000);

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
             robot.actuator.setPower(0.5);
            while (opModeIsActive() && bottom > 1.86) {
              bottom = bottom();
              robot.actuator.setPower(Math.max(0.75, bottom));
             telemetry.addData("bottom", bottom);
             telemetry.update();
            }
            sleep(100);
            robot.actuator.setPower(0);


            moveTank(0.6, -3, -3, 3);
            sleep(100);
            strafe(0.4, 2, 2);
            sleep(100);
            moveTank(0.6, 2, 2, 3);
            sleep(100);

            strafe(STRAFE_SPEED, 4, 5);
            sleep(100);

            if (position == -1) {
                rotate(-60, 0.5);
            } else if (position == 1) {
                rotate(-130, 0.5);
            } else {
                rotate(-95, 0.5);
            }

            // drop intake
            robot.intakeliftleft.setPosition(1.0);
            robot.intakeliftright.setPosition(1.0);
            sleep(1000);
            // spin in
            robot.intake.setPower(-0.3);
            sleep(1000);
            // extend intake
            robot.intakeextension.setPower(-1);
            sleep(1000);
            robot.intake.setPower(0);
            robot.intakeextension.setPower(0);
            sleep(1000);
            // raise intake
            robot.intakeliftleft.setPosition(0.3);
            robot.intakeliftright.setPosition(0.3);
            // retract intake
            robot.intakeextension.setPower(1);
            sleep(1000);
            robot.intake.setPower(0);


            if (position == -1) {
                rotate(-30, 0.5);
            } else if (position == 1) {
                rotate(40, 0.5);
            } else {
                rotate(-5, 0.5);
            }

//
//            //90 dergee turn
//            moveTank(0.6, 10.5, -10.5, 5, false);
//
//            if(position == -1){//left
//                strafe(STRAFE_SPEED, -16, 10, false);
//                moveTank(0.6, 8, 8, 6, false);
//            }if(position == 0){//center
//                moveTank(0.6, 4, 4, 5, false);
//            }else{//right
//                strafe(STRAFE_SPEED, 15, 10, false);
//                moveTank(0.6, 8, 8, 6, false);
//            }


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

    public void strafe(double speed,
                              double inches,
                              double timeoutS) {

            int newFrontLeftTarget, newBackLeftTarget, newFrontRightTarget, newBackRightTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newFrontLeftTarget = robot.leftfront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.leftback.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                newFrontRightTarget = robot.rightfront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.rightback.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

                robot.leftfront.setTargetPosition(newFrontLeftTarget);
                robot.leftback.setTargetPosition(newBackLeftTarget);
                robot.rightfront.setTargetPosition(newFrontRightTarget);
                robot.rightback.setTargetPosition(newBackRightTarget);

                // Turn On RUN_TO_POSITION
                robot.rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.leftback.setPower(Math.abs(speed) * Math.signum(inches));
                robot.rightback.setPower(Math.abs(speed) * Math.signum(inches));
                robot.leftfront.setPower(Math.abs(speed) * Math.signum(inches));
                robot.rightfront.setPower(Math.abs(speed) * Math.signum(inches));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.rightback.isBusy() && robot.rightfront.isBusy()
                                && robot.leftback.isBusy() && robot.leftfront.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Current",  "Running to %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                    telemetry.addData("backRight",  "Running at bl: %7d, fl: %7d, br: %7d, fr: %7d",
                            robot.leftback.getCurrentPosition(),
                            robot.leftfront.getCurrentPosition(),
                            robot.rightback.getCurrentPosition(),
                            robot.rightfront.getCurrentPosition());

                    telemetry.update();
                }

                // Stop all motion;
                robot.rightfront.setPower(0);
                robot.leftfront.setPower(0);
                robot.rightback.setPower(0);
                robot.leftback.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        public void moveTank(double speed,
                                 double leftInches, double rightInches,
                                 double timeoutS) {

                int newFrontLeftTarget, newBackLeftTarget, newFrontRightTarget, newBackRightTarget;

                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                    // Determine new target position, and pass to motor controller
                    newFrontLeftTarget = robot.leftfront.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
                    newBackLeftTarget = robot.leftback.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
                    newFrontRightTarget = robot.rightfront.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
                    newBackRightTarget = robot.rightback.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);

                    robot.leftfront.setTargetPosition(newFrontLeftTarget);
                    robot.leftback.setTargetPosition(newBackLeftTarget);
                    robot.rightfront.setTargetPosition(newFrontRightTarget);
                    robot.rightback.setTargetPosition(newBackRightTarget);

                    // Turn On RUN_TO_POSITION
                    robot.rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // reset the timeout time and start motion.
                    runtime.reset();
                    robot.leftback.setPower(Math.abs(speed));
                    robot.rightback.setPower(Math.abs(speed));
                    robot.leftfront.setPower(Math.abs(speed));
                    robot.rightfront.setPower(Math.abs(speed));

                    // keep looping while we are still active, and there is time left, and both motors are running.
                    // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                    // its target position, the motion will stop.  This is "safer" in the event that the robot will
                    // always end the motion as soon as possible.
                    // However, if you require that BOTH motors have finished their moves before the robot continues
                    // onto the next step, use (isBusy() || isBusy()) in the loop test.
                    while (opModeIsActive() &&
                            (runtime.seconds() < timeoutS) &&
                            (robot.rightback.isBusy() && robot.rightfront.isBusy()
                                    && robot.leftback.isBusy() && robot.leftfront.isBusy())) {

                        // Display it for the driver.
                        telemetry.addData("Current",  "Running to %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                        telemetry.addData("backRight",  "Running at bl: %7d, fl: %7d, br: %7d, fr: %7d",
                                robot.leftback.getCurrentPosition(),
                                robot.leftfront.getCurrentPosition(),
                                robot.rightback.getCurrentPosition(),
                                robot.rightfront.getCurrentPosition());

                        telemetry.update();
                    }

                    // Stop all motion;
                    robot.rightfront.setPower(0);
                    robot.leftfront.setPower(0);
                    robot.rightback.setPower(0);
                    robot.leftback.setPower(0);

                    // Turn off RUN_TO_POSITION
                    robot.rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }

    public void rotate(double degrees, double maxSpeed) {
        double K_P = 0.0502;
        double K_I = 0;
        double K_D = 0.0014;

        double target = degrees - heading();
        double error = 100;
        double lastError = 0;
        double integral = 0;
        double derivative = 0;
        ArrayList<Double> values = new ArrayList<Double>();

        double leftPower = 1, rightPower;
        while (opModeIsActive() && (values.size() <= 10 || Math.abs(values.get(values.size()-1-10) -
                values.get(values.size()-1)) > 0.01 )) {
            error = computeError(heading(), target);

            values.add(error);

            integral += error;
            derivative = error - lastError;

            leftPower = Math.max(Math.min(K_P * error + K_I * integral + K_D * derivative, maxSpeed), -maxSpeed);
            rightPower = -leftPower;
            robot.leftfront.setPower(leftPower);
            robot.leftback.setPower(leftPower);
            robot.rightfront.setPower(rightPower);
            robot.rightback.setPower(rightPower);
            telemetry.addData("error", error);
            telemetry.addData("proportional", K_P * error);
            telemetry.addData("integral", K_I * integral);
            telemetry.addData("derivative", K_D * derivative);
            telemetry.addData("leftPower", leftPower);
            telemetry.addData("rightPower", rightPower);
            telemetry.update();
            lastError = error;
        }
        robot.leftback.setPower(0);
        robot.leftfront.setPower(0);
        robot.rightback.setPower(0);
        robot.rightfront.setPower(0);

    }

    private double heading() {
        return -robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private double computeError(double heading, double target) {
        double error = target + heading;
        if (error > 180) {
            error = error - 360;
        }
        return error;
    }

}
