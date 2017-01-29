/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Robot Drive", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class RobotLinearDrive extends LinearOpMode {

    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;

    ModernRoboticsI2cGyro gyro = null;

    DcMotor shooter = null;
    Servo placer = null;
    OpticalDistanceSensor shootDetector = null;


    Servo selectionRight = null;
    Servo selectionLeft = null;

    double fastSpeed = 0.8;
    double slowSpeed = 0.4;

    private ElapsedTime runtime = new ElapsedTime();

    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Test1");
            float tankRight = -gamepad1.right_stick_y;
            float tankLeft = -gamepad1.left_stick_y;

            telemetry.addData("Status", "%.2f", tankLeft);
            telemetry.addData("Status", "%.2f", tankRight);

            if (gamepad1.dpad_up) {
                leftBack.setPower(fastSpeed);
                rightBack.setPower(fastSpeed);
                leftFront.setPower(fastSpeed);
                rightFront.setPower(fastSpeed);
            } else if (gamepad1.dpad_down) {
                leftBack.setPower(-fastSpeed);
                rightBack.setPower(-fastSpeed);
                leftFront.setPower(-fastSpeed);
                rightFront.setPower(-fastSpeed);
            } else if (gamepad1.dpad_left) {
                leftBack.setPower(fastSpeed);
                rightBack.setPower(fastSpeed);
                leftFront.setPower(-fastSpeed);
                rightFront.setPower(-fastSpeed);
            } else if (gamepad1.dpad_right) {
                leftBack.setPower(-fastSpeed);
                rightBack.setPower(-fastSpeed);
                leftFront.setPower(fastSpeed);
                rightFront.setPower(fastSpeed);

            } else if (gamepad1.y) {
                leftBack.setPower(slowSpeed);
                rightBack.setPower(slowSpeed);
                leftFront.setPower(slowSpeed);
                rightFront.setPower(slowSpeed);
            } else if (gamepad1.a) {
                leftBack.setPower(-slowSpeed);
                rightBack.setPower(-slowSpeed);
                leftFront.setPower(-slowSpeed);
                rightFront.setPower(-slowSpeed);
            } else if (gamepad1.x) {
                leftBack.setPower(slowSpeed);
                rightBack.setPower(slowSpeed);
                leftFront.setPower(-slowSpeed);
                rightFront.setPower(-slowSpeed);
            } else if (gamepad1.b) {
                leftBack.setPower(-slowSpeed);
                rightBack.setPower(-slowSpeed);
                leftFront.setPower(slowSpeed);
                rightFront.setPower(slowSpeed);

            } else {

                leftBack.setPower(tankLeft);
                rightFront.setPower(tankRight);
                rightBack.setPower(tankRight);
                leftFront.setPower(tankLeft);
            }

            telemetry.update();
            idle();
        }
    }
    public void initializeRobot(){

    }
}




