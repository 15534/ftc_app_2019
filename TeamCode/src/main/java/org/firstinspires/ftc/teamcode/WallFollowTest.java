package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name = "Wall Follow Test", group = "World Bot")
public class WallFollowTest extends LinearOpMode {

    public HardwareWorldsRobot robot = new HardwareWorldsRobot();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
        waitForStart();

        double front, back, theta, leftPower, rightPower, bottom;
        final double speed = 0.2;
        while (opModeIsActive()) {
            front = robot.rightfrontsensor.getDistance(DistanceUnit.INCH);
            back = robot.rightbacksensor.getDistance(DistanceUnit.INCH);
            bottom = robot.bottomsensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("rightfront", front);
            telemetry.addData("rightback", back);
//            telemetry.addData("back", robot.backsensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("bottom", robot.bottomsensor.getDistance(DistanceUnit.INCH));
            theta = theta(front, back);
            telemetry.addData("theta", theta);
            telemetry.addData("bottom", bottom);

            telemetry.update();
        }
    }

    double theta (double front, double back) {
        return Math.atan((back-front) / robot.distanceBetweenRightDistance) * 360 / (2 * Math.PI);
    }


}
