package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HardwareWorldsRobot {

    public DcMotor rightfront;
    public DcMotor leftfront;
    public DcMotor rightback;
    public DcMotor leftback;

    public Servo phone;

    public DcMotor intakeextension;
    public DcMotor dumperextension;
    public DcMotor actuator;
    public Servo dumper;
    public CRServo intake;
    public Servo intakeliftleft;
    public Servo intakeliftright;
    public BNO055IMU imu;
    public final double distanceBetweenRightDistance = 36.85 / 2.54;

    public Rev2mDistanceSensor rightfrontsensor, rightbacksensor, bottomsensor, backsensor;


    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Initialize drive motors
        leftfront = hwMap.get(DcMotor.class, "leftfront");
        rightfront = hwMap.get(DcMotor.class, "rightfront");
        leftback = hwMap.get(DcMotor.class, "leftback");
        rightback = hwMap.get(DcMotor.class, "rightback");
        actuator = hwMap.get(DcMotor.class, "actuator");
        intake = hwMap.get(CRServo.class, "intake");
        intakeliftleft = hwMap.get(Servo.class, "liftleft");
        intakeliftright = hwMap.get(Servo.class, "liftright");

        leftfront.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.FORWARD);
        actuator.setDirection(DcMotor.Direction.REVERSE);

        leftfront.setPower(0);
        rightfront.setPower(0);
        leftback.setPower(0);
        rightback.setPower(0);
        actuator.setPower(0);

        // Initialize dumper servo
        dumper = hwMap.get(Servo.class, "dumper");

        phone = hwMap.get(Servo.class, "phone");
        phone.setDirection(Servo.Direction.REVERSE);

        // Initialize extension motors
        dumperextension = hwMap.get(DcMotor.class, "dumperextension");
        dumperextension.setPower(0);
        dumperextension.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeextension = hwMap.get(DcMotor.class, "intakeextension");
        intakeextension.setPower(0);
        intakeextension.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeliftleft.setDirection(Servo.Direction.FORWARD);
        intakeliftright.setDirection(Servo.Direction.REVERSE);
        dumper.setDirection(Servo.Direction.REVERSE);

        // initialize distance sensors
        rightfrontsensor = hwMap.get(Rev2mDistanceSensor.class, "rightFront");
        rightbacksensor = hwMap.get(Rev2mDistanceSensor.class, "rightBack");
        backsensor = hwMap.get(Rev2mDistanceSensor.class, "back");
        bottomsensor = hwMap.get(Rev2mDistanceSensor.class, "bottom");


        // We are expecting the IMU to be attached to an I2C port on a Core Device Interface Module and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
}
