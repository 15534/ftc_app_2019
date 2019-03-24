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
        while (opModeIsActive()) {
            telemetry.addData("rightfront", robot.rightfrontsensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("rightback", robot.rightbacksensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("back", robot.backsensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("bottom", robot.bottomsensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
