package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

@TeleOp(name = "Worlds Teleop", group = "World Bot")
public class WorldsTeleop  extends LinearOpMode {

    public HardwareWorldsRobot robot = new HardwareWorldsRobot();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.8;
    static final double     STRAFE_SPEED            = 0.4;
    static final double     DETECT_SPEED            = 0.1;
    static final double     PUSH_SPEED              = 0.2;
    static final double     intakePeriod            = 1.2;
//    static final double     intakePeriod            = 0.5;
    static final long       MOVE_DELAY              = 250;

    private final double DPAD_SPEED = 0.45;

    private final int INITIAL_POWER = 8;
    private final int INCREMENT_POWER = 2; // How much power to increase / decrease every time the dpad button is pressed
    private final double DEBUG_MODE_POWER_MULTIPLIER = 0.5; // Make the entire robot move slower when debugging, not sure whether useful or not
    private final double INTAKE_POWER_ADJUST = 0.4;

    private double leftBackPower = 0, rightBackPower = 0, leftFrontPower = 0, rightFrontPower = 0;
    private double dumperExtensionPower = 0;
    private double intakeExtensionPower = 0;
    private double intake;
//    double intakelift;

    enum GameMode {
        MODE_1,
        MODE_2
    }

    private int power = INITIAL_POWER; // integer from 1 to 10

    private GameMode gameMode = GameMode.MODE_2;
    private boolean canChangeGuide = true;
    private boolean canChangePower = true;

    private boolean debugMode = false;
    private ElapsedTime runtime = new ElapsedTime();
    private int intakeDirection = 0;  // -1: intake, 1: release;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        telemetry.addData(">", "Init Successful");
        telemetry.update();
        waitForStart();
        robot.intakeextension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        while (opModeIsActive()) {
            if (gamepad1.start && canChangeGuide) {
                gameMode = nextMode(gameMode);
                canChangeGuide = false;
            } else if (!gamepad1.start && !canChangeGuide) {
                // button unpressed, need to now allow for button presses
                canChangeGuide = true;
            }

            if (gameMode == GameMode.MODE_1) {
                // Check for Power change
                if (gamepad1.dpad_up && canChangePower) {
                    power = nextPower(power);
                    canChangePower = false;
                } else if (gamepad1.dpad_down && canChangePower) {
                    power = previousPower(power);
                    canChangePower = false;
                } else if (!gamepad1.dpad_up && !gamepad1.dpad_down && !canChangePower) {
                    canChangePower = true;
                }

                // Drivetrain: move left and right
                moveLeft((double) power / 10 * gamepad1.left_stick_y);
                moveRight((double) power / 10 * gamepad1.right_stick_y);

                // Strafing
                if (gamepad1.right_bumper) {
                    strafe(-(double) power / 10);
                } else if (gamepad1.left_bumper) {
                    strafe((double) power / 10);
                }
            }else if (gameMode == GameMode.MODE_2) {
                stopMotors();
                if (gamepad1.dpad_down) {
                    moveLeft(DPAD_SPEED);
                    moveRight(DPAD_SPEED);
                } else if (gamepad1.dpad_up) {
                    moveLeft(-DPAD_SPEED);
                    moveRight(-DPAD_SPEED);
                } else if (gamepad1.dpad_left) {
                    strafe(DPAD_SPEED);
                } else if (gamepad1.dpad_right) {
                    strafe(-DPAD_SPEED);
                }

                if (gamepad1.right_bumper) {
                    rotate(DPAD_SPEED);
                } else if (gamepad1.left_bumper) {
                    rotate(-DPAD_SPEED);
                }

                if (gamepad1.left_stick_y > 0.2) {
                    robot.dumper.setPosition(1);
                } else if (gamepad1.left_stick_y < -0.2) {
                    robot.dumper.setPosition(0);
                }

                dumperExtensionPower = -1 * gamepad1.right_stick_y;

            }

            if (gamepad2.left_stick_y > 0.2) {
                robot.intakeliftright.setPosition(0);
                robot.intakeliftleft.setPosition(0);
            } else if (gamepad2.left_stick_y < -0.2) {
                robot.intakeliftright.setPosition(1);
                robot.intakeliftleft.setPosition(1);
            }

            if (gamepad2.right_bumper) {
                rotate(DPAD_SPEED);
            } else if (gamepad2.left_bumper) {
                rotate(-DPAD_SPEED);
            }

            if (gamepad1.y) {
                robot.actuator.setPower(1);
            } else if (gamepad1.x) {
                robot.actuator.setPower(-1);
            } else {
                robot.actuator.setPower(0);
            }

            intakeExtensionPower = gamepad2.right_stick_y;
            intake = -INTAKE_POWER_ADJUST * gamepad2.left_stick_y;

            if (gamepad2.a) {
                intakeDirection = -1;
            }
            if (gamepad2.y) {
                intakeDirection = -2;
            }
            if (gamepad2.b){
                intakeDirection = 1;
            }
            if (gamepad2.x){
                intakeDirection = 0;
            }

            if(intakeExtensionPower > 0){
                intakeDirection = 0;
            }

            updateMotors();
            updateTelemetry();
        }
        }

    private void moveLeft(double power) {
        leftFrontPower = leftBackPower = power;
    }

    private void moveRight(double power) {
        rightBackPower = rightFrontPower = power;
    }

    private void strafe(double power) {
        leftFrontPower = rightBackPower = power;
        rightFrontPower = leftBackPower = -power;
    }

    private void rotate(double power) {
        rightFrontPower = rightBackPower = power;
        leftFrontPower = leftBackPower = -power;
    }


    private void stopMotors() {
        leftFrontPower = rightBackPower = leftBackPower = rightFrontPower = 0;
    }

    private void updateTelemetry() {
        telemetry.addData("liftleft", robot.intakeliftleft.getPosition());
        telemetry.addData("liftright", robot.intakeliftright.getPosition());
        telemetry.addData("Gamepad1", gamepad1.toString());
        telemetry.addData("Gamepad2", gamepad2.toString());
        telemetry.addData("Game Mode", modeName(gameMode));
        telemetry.addData("power adjust", String.format(Locale.ENGLISH, "%.2f", (double) power / 10));
        telemetry.addData("dumperExtensionPower", String.format(Locale.ENGLISH, "%.2f", dumperExtensionPower));
        telemetry.addData("leftBackPower", String.format(Locale.ENGLISH, "%.2f", leftBackPower));
        telemetry.addData("rightBackPower", String.format(Locale.ENGLISH, "%.2f", rightBackPower));
        telemetry.addData("leftFrontPower", String.format(Locale.ENGLISH, "%.2f", leftFrontPower));
        telemetry.addData("rightFrontPower", String.format(Locale.ENGLISH, "%.2f", rightFrontPower));
        telemetry.addData("intakeLeftPower", String.format(Locale.ENGLISH, "%.2f", robot.intake.getPower()));
        telemetry.update();
    }

    private void updateMotors() {
        if (debugMode) {
            robot.leftfront.setPower(leftFrontPower * DEBUG_MODE_POWER_MULTIPLIER);
            robot.leftback.setPower(leftBackPower * DEBUG_MODE_POWER_MULTIPLIER);
            robot.rightfront.setPower(rightFrontPower * DEBUG_MODE_POWER_MULTIPLIER);
            robot.rightback.setPower(rightBackPower * DEBUG_MODE_POWER_MULTIPLIER);
            robot.dumperextension.setPower(dumperExtensionPower * DEBUG_MODE_POWER_MULTIPLIER);
//            robot.intakeright.setPower(intakeRightPower * DEBUG_MODE_POWER_MULTIPLIER);
//            robot.intakeleft.setPower(intakeLeftPower * DEBUG_MODE_POWER_MULTIPLIER);

        } else {
            robot.leftfront.setPower(leftFrontPower);
            robot.leftback.setPower(leftBackPower);
            robot.rightfront.setPower(rightFrontPower);
            robot.rightback.setPower(rightBackPower);
            robot.dumperextension.setPower(dumperExtensionPower);
//            robot.intakeright.setPower(intakeRightPower);
//            robot.intakeleft.setPower(intakeLeftPower);
            robot.intakeextension.setPower(intakeExtensionPower);
        }

        switch (intakeDirection) {
            case 0:
                robot.intake.setPower(0);
                break;
            case -1:
//                if (runtime.seconds() % intakePeriod < (intakePeriod * 0.8))
//                    robot.intake.setPower(-0.8);
//                else
//                    robot.intake.setPower(0);
//                break;
                robot.intake.setPower((runtime.seconds() % intakePeriod) / intakePeriod * -0.8);
//                if (runtime.seconds() % intakePeriod < (intakePeriod * 0.8))
//                    robot.intake.setPower((runtime.seconds() % intakePeriod) / intakePeriod * -0.8);
//                else
//                    robot.intake.setPower(0);
                break;
            case -2:
                robot.intake.setPower(-0.1);
                break;
            case 1:
                robot.intake.setPower(0.8);
                break;
        }


    }

    private GameMode nextMode(GameMode m) {
        // Can eventually accommodate more than 2 modes

        if (m == GameMode.MODE_1) return GameMode.MODE_2;
        if (m == GameMode.MODE_2) return GameMode.MODE_1;

        return null;
    }

    private int previousPower(int p) {
        if (p - INCREMENT_POWER <= 0) return 10;
        return p - INCREMENT_POWER;
    }

    private int nextPower(int p) {
        if (p + INCREMENT_POWER > 10) return 2;
        return p + INCREMENT_POWER;
    }

    private String modeName(GameMode m) {
        if (m == GameMode.MODE_1) return "1";
        if (m == GameMode.MODE_2) return "2";
        return null;
    }

}
