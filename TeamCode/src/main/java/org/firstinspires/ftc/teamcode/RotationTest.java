/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * {@link RotationTest} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@TeleOp(name = "PID Gyro Tuning", group = "Sensor")
public class RotationTest extends LinearOpMode
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object

    // State used for updating telemetry
    Orientation angles;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    public HardwareWorldsRobot robot = new HardwareWorldsRobot();


        @Override public void runOpMode() {

            robot.init(hardwareMap);

            // Wait until we're told to go
            waitForStart();

            // Start the logging of measured acceleration
            robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

            // Loop and update the dashboard
            double heading;
            boolean running = false;
            double K_P = 0.0502; // 0.0502
            double K_I = 0;
            double K_D = 0;

            int editing = 0;

            while (opModeIsActive()) {

                if (gamepad1.dpad_left) {
                    editing = (editing + 2) % 3;
                    sleep(200);
                }
                if (gamepad1.dpad_right) {
                    editing = (editing + 1) % 3;
                    sleep(200);
                }
                if (gamepad1.dpad_up) {
                    if (editing == 0) {
                        K_P += 0.001;
                    } else if (editing == 1){
                        K_I += 0.0001;
                    } else if (editing == 2) {
                        K_D += 0.0001;
                    }
                    sleep(200);
                }
                if (gamepad1.dpad_down) {
                    if (editing == 0) {
                        K_P -= 0.001;
                    } else if (editing == 1){
                        K_I -= 0.001;
                    } else if (editing == 2) {
                        K_D -= 0.001;
                    }
                    sleep(200);
                }


                if (editing == 0) {
                    telemetry.addData(">kp", K_P);
                    telemetry.addData("ki", K_I);
                    telemetry.addData("kd", K_D);
                } else if (editing == 1) {
                    telemetry.addData("kp", K_P);
                    telemetry.addData(">ki", K_I);
                    telemetry.addData("kd", K_D);
                } else {
                    telemetry.addData("kp", K_P);
                    telemetry.addData("ki", K_I);
                    telemetry.addData(">kd", K_D);
                }

                telemetry.update();
                if (gamepad1.a) {
                    rotate(90, 1.0, K_P, K_I, K_D);
                }
            }
    }

    public void rotate(double degrees, double maxSpeed, double K_P, double K_I, double K_D) {
        double target = degrees - heading();
        double error;
        double lastError = 0;
        double integral = 0;
        double derivative = 0;


        double leftPower, rightPower;
        while (opModeIsActive() && !(gamepad1.x)) {
            error = computeError(heading(), target);
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
