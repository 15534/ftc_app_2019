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
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;

import java.sql.Driver;
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

    static final double     STRAFE_SPEED            = 0.9;
    static final double     MOVE_SPEED              = 0.9;
    static final double     TURN_SPEED              = 0.6;


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

//        dump routine
//        robot.dumper.setPosition(1);
//        sleep(500);
//        // bring down while getting mineral
//        robot.dumperextension.setPower(-0.5);
//        sleep(400);
//        robot.dumperextension.setPower(0);
//        robot.intake.setPower(-0.3);
//        sleep(1000);
//        robot.intake.setPower(0);
//        robot.dumperextension.setPower(1);
//        sleep(1000);
//        robot.dumperextension.setPower(0);
//
//
//        robot.dumper.setPosition(0);

        if (opModeIsActive()) {
            robot.dumperextension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.intakeliftleft.setPosition(1.0);
            robot.intakeliftright.setPosition(1.0);
            sleep(200);
            robot.dumper.setPosition(1);
            sleep(200);
            robot.phone.setPosition(0);
            sleep(500);
            int position = -1;
            int goldMineralY = -1;

            runtime.reset();

            // detect mineral position
            while (opModeIsActive() && runtime.seconds() < 1) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    int goldMineralX = -1;
                    if (updatedRecognitions != null) {
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) && recognition.getBottom() > 200) {
                                    goldMineralX = (int) recognition.getLeft();
                                    goldMineralY = (int) recognition.getBottom();
                            }
                        }

                        if (goldMineralX > 830 && goldMineralX < 1300) position = 1;
                        if (goldMineralX < 830 && goldMineralX > 300) position = 0;
                    }

                    telemetry.addData("top", goldMineralY);
                    telemetry.addData("position", position);
                    telemetry.update();
            }

            if (tfod != null) {
                tfod.shutdown();
            }

            double bottom = 10;
             runtime.reset();
            while (opModeIsActive() && bottom > 0 && runtime.seconds() < 3) {
              bottom = bottom();
              robot.actuator.setPower(1);

                if (position == -1) {
                    telemetry.addData("mineral", "left");
                }
                else if (position == 0) {
                    telemetry.addData("mineral", "center");
                }
                else {
                    telemetry.addData("mineral", "right");
                }

             telemetry.addData("bottom", bottom);
             telemetry.update();
            }
            sleep(300);
            robot.actuator.setPower(0);

            moveTank(MOVE_SPEED, -3, -3, 3);
            strafe(MOVE_SPEED, 4, 2);
            moveTank(MOVE_SPEED, 2, 2, 3);
//
//            strafe(STRAFE_SPEED, 2, 5);
//            sleep(100);

            if (position == -1) {
                rotate(60, 0.5);
            } else if (position == 1) {
                rotate(120, 0.5);
            } else {
                rotate(90, 0.5);
            }

            // drop intake
//            robot.intakeliftleft.setPosition(1.0);
//            robot.intakeliftright.setPosition(1.0);
//            sleep(1000);
            // spin in
            robot.intake.setPower(-0.3);
            // extend intake
            robot.intakeextension.setPower(-1);
            sleep(400);
            robot.dumperextension.setPower(-0.5);
            sleep(400);
            robot.dumperextension.setPower(0);
            sleep(200);
            robot.intake.setPower(0);
            robot.intakeextension.setPower(0);
//            sleep(200);
            // raise intake
            robot.intakeliftleft.setPosition(0.3);
            robot.intakeliftright.setPosition(0.3);
            // retract intake
            robot.intakeextension.setPower(1);
            sleep(500);

            IntakeMineralExtension intakeMineralExtension = new IntakeMineralExtension();
            intakeMineralExtension.start();

            if (position == -1) {
                rotate(30, 0.5);
            } else if (position == 1) {
                rotate(-30, 0.5);
            } else {
//                rotate(-10, 0.5);
            }

            moveTank(0.5, 6, 6, 5);

//            sleep(200);

            rotate(90, 0.5);
            moveTank(MOVE_SPEED, -36, -36, 10);
            encoderRotate(-48, TURN_SPEED);
            strafeTime(-0.5, 500);
            strafe(STRAFE_SPEED, 2, 5);
            moveTank(MOVE_SPEED, -24, -24, 10);
            double distance = robot.backsensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("distance", distance);
            telemetry.update();
            moveTank(MOVE_SPEED, -(distance - 21), -(distance-21), 5);
            robot.dumperextension.setPower(1);
            sleep(500);
            robot.dumperextension.setPower(0);
            moveTank(MOVE_SPEED, 30, 30, 10);
            strafe(STRAFE_SPEED, 7, 10);
            ExtendDumper extendDumper = new ExtendDumper();
            extendDumper.start();
            encoderRotate(-80, MOVE_SPEED);
            sleep(100);
            moveTank(MOVE_SPEED, -6, -6, 5);
            robot.dumper.setPosition(0);
            sleep(1200);

            LowerDumperExtension lowerDumperExtension = new LowerDumperExtension();
            lowerDumperExtension.start();
            encoderRotate(70, 1);

            MoveRobotForward moveRobotForward = new MoveRobotForward();
            moveRobotForward.start();
            robot.intakeextension.setPower(-1);
            sleep(500);
            robot.intakeextension.setPower(0);

            // drop intake
            robot.intakeliftleft.setPosition(1.0);
            robot.intakeliftright.setPosition(1.0);

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
        double bottom = robot.bottomsensor.getDistance(DistanceUnit.INCH) - 2;
        return bottom;
    }

    private double rightFront() {
        return robot.rightfrontsensor.getDistance(DistanceUnit.INCH);
    }

    private double rightBack() {
        return robot.rightbacksensor.getDistance(DistanceUnit.INCH);
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

    public void strafeTime(double speed, long timeoutMS) {
        robot.leftback.setPower(speed);
        robot.rightfront.setPower(speed);
        robot.rightback.setPower(-speed);
        robot.leftfront.setPower(-speed);
        sleep(timeoutMS);
        robot.leftfront.setPower(0);
        robot.leftback.setPower(0);
        robot.rightback.setPower(0);
        robot.rightfront.setPower(0);
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

                sleep(100);
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

                    sleep(100);
                }

            }

    public void encoderRotate(double degrees, double speed) {
        double leftInches = degrees / 360 * 2 * Math.PI * 9.3;
        moveTank(speed, leftInches, -leftInches, 5);
    }

    public void rotate(double degrees, double maxSpeed) {
        double K_P = -0.0502;
        double K_I = 0;
//        double K_D = 0.0014;
        double K_D = 0;

        double target = degrees + heading();
        encoderRotate(degrees, 1);

        double error = 100;
        double lastError = 0;
        double integral = 0;
        double derivative = 0;
        ArrayList<Double> values = new ArrayList<Double>();

        double leftPower = 1, rightPower;
        double predPower = 0;
        while (opModeIsActive()
                && ((values.size() <= 5 || Math.abs(values.get(values.size()-1-5) -
                values.get(values.size()-1)) > 0.01) && (Math.abs(error) > 0.1))
            ) {
            error = computeError(heading(), target);

            values.add(error);

            integral += error;
            derivative = error - lastError;

            predPower = K_P * error + K_I * integral + K_D * derivative;
            if (Math.abs(predPower) > Math.abs(maxSpeed)) {
                predPower = predPower / Math.abs(predPower) * maxSpeed;
            }

            leftPower = predPower;
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

        sleep(100);

    }

    private double heading() {
        return -robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private double computeError(double heading, double target) {
        double error = target - heading;
        if (error > 180) {
            error = error - 360;
        }
        return error;
    }

    class LowerDumperExtension implements Runnable {
        private Thread t;
        private String threadName;

        LowerDumperExtension() {
            threadName = "a";
            System.out.println("Creating " +  threadName );
        }

        public void run() {
            System.out.println("Running " +  threadName );
            robot.dumperextension.setPower(-0.5);
            sleep(1000);
            robot.dumperextension.setPower(0);
        }

        public void start() {
            if (t == null) {
                t = new Thread (this, threadName);
                t.start ();
            }
        }
    }

    class IntakeMineralExtension implements Runnable {
        private Thread t;
        private String threadName;

        IntakeMineralExtension() {
            threadName = "a";
            System.out.println("Creating " +  threadName );
        }

        public void run() {
            robot.intakeextension.setPower(1);
            sleep(200);
            robot.intakeextension.setPower(0);
            robot.intake.setPower(0);
            robot.intakeextension.setPower(0.5);
            sleep(1000);
            robot.intakeextension.setPower(0);
            robot.intake.setPower(-0.3);
            sleep(1000);
            robot.intake.setPower(0);
            robot.intakeextension.setPower(0);
        }

        public void start() {
            if (t == null) {
                t = new Thread (this, threadName);
                t.start ();
            }
        }
    }

    class MoveRobotForward implements Runnable {
        private Thread t;
        private String threadName;

        MoveRobotForward() {
            threadName = "a";
            System.out.println("Creating " +  threadName );
        }

        public void run() {
            moveTank(MOVE_SPEED, 6, 6, 5);
        }

        public void start() {
            if (t == null) {
                t = new Thread (this, threadName);
                t.start ();
            }
        }
    }

    class ExtendDumper implements Runnable {
        private Thread t;
        private String threadName;

        ExtendDumper() {
            threadName = "a";
            System.out.println("Creating " +  threadName );
        }

        public void run() {
            robot.dumperextension.setPower(1);
            sleep(1000);
            robot.dumperextension.setPower(0);
        }

        public void start() {
            if (t == null) {
                t = new Thread (this, threadName);
                t.start ();
            }
        }
    }



}
