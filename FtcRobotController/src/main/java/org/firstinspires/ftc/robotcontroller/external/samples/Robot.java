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

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Template: Iterative OpMode", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class Robot extends LinearOpMode {
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;

    ModernRoboticsI2cGyro gyro = null;

    DcMotor shooter = null;
    Servo placer = null;
    OpticalDistanceSensor shootDetector = null;

    ColorSensor beaconColorleft = null;
    ColorSensor beaconColorright = null;

    ColorSensor leftColorAlign = null;
    ColorSensor rightColorAlign = null;
    OpticalDistanceSensor wallFollowing = null;

    Servo selectionRight = null;
    Servo selectionLeft = null;

    private ElapsedTime runtime = new ElapsedTime();
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {
    }


    public void initializeRobot() throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        //gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        //leftColorAlign = hardwareMap.colorSensor.get("leftColorAlign");
        //rightColorAlign = hardwareMap.colorSensor.get("rightColorAlign");
        //leftColorAlign.setI2cAddress(I2cAddr.create8bit(0x3c));
        //rightColorAlign.setI2cAddress(I2cAddr.create8bit(0x10));
        //leftColorAlign.enableLed(true);
        //rightColorAlign.enableLed(true);
    }

    public void SetRun_To_Position(){
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void resetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runWithoutEncoders(){
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void runWithEncoders(){
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void EncoderDrive(double speed, int inchesRight, int inchesLeft) throws InterruptedException {
        runWithEncoders();
        int targetRight = inchesRight * (int) COUNTS_PER_INCH;
        int targetLeft = inchesLeft * (int) COUNTS_PER_INCH;
        leftFront.setTargetPosition(targetLeft + leftFront.getCurrentPosition());
        rightFront.setTargetPosition(targetRight + rightFront.getCurrentPosition());
        leftBack.setTargetPosition(targetLeft + leftBack.getCurrentPosition());
        rightBack.setTargetPosition(targetRight + rightBack.getCurrentPosition());
        SetRun_To_Position();
        if (targetLeft > 0) {
            leftFront.setPower(speed);
            leftBack.setPower(speed);
        } else {
            leftFront.setPower(-speed);
            leftBack.setPower(-speed);
        }
        if (targetRight > 0) {
            rightFront.setPower(speed);
            rightBack.setPower(speed);
        } else {
            rightFront.setPower(-speed);
            rightBack.setPower(-speed);
        }
        while ((leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) && opModeIsActive()) {
            idle();
            sleep(5);
        }
        stopDrive();

    }

    public void stopDrive() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void driveWithTime(double speed, long timeInMili) throws InterruptedException {
        runWithoutEncoders();
        leftBack.setPower(speed);
        leftFront.setPower(speed);
        rightBack.setPower(speed);
        sleep(timeInMili);
        stopDrive();
    }

    public void turnWithTime(double rightSpeed, double leftSpeed, long timeInMili) throws InterruptedException {
        runWithoutEncoders();
        rightFront.setPower(rightSpeed);
        rightBack.setPower(rightSpeed);
        leftFront.setPower(leftSpeed);
        leftBack.setPower(leftSpeed);
        sleep(timeInMili);
        stopDrive();
    }

    public String getBeaconColor(ColorSensor color) {
        if (color.red() > color.blue() && color.red() > color.green()) {
            return "Red";
        }
        if (color.blue() > color.red() && color.blue() > color.green()) {
            return "Blue";
        } else {
            return "None";
        }
    }

    public String getLineColor(ColorSensor color) { //need to create function using test results
        return "";
    }

    public void driveUntilLine(double speed) {
        runWithoutEncoders();
        String leftColor = getLineColor(leftColorAlign);
        String rightColor = getLineColor(rightColorAlign);
        while (!leftColor.equals("White") && !rightColor.equals("White")) {
            if (leftColor.equals("White")) {
                leftFront.setPower(0);
                leftBack.setPower(0);
            } else {
                leftFront.setPower(speed);
                leftBack.setPower(speed);
            }

            if (rightColor.equals("White")) {
                rightFront.setPower(0);
                rightBack.setPower(0);
            }else{
                rightFront.setPower(speed);
                rightBack.setPower(speed);
            }
        }
    }

    public void turnAbsolute(int target, double speed) throws InterruptedException{
        runWithoutEncoders();
        int currentVal = gyro.getIntegratedZValue();
        while(Math.abs(currentVal - target) > 3){

            if(currentVal > target){
                leftFront.setPower(speed);
                leftBack.setPower(speed);
                rightFront.setPower(-speed);
                rightBack.setPower(-speed);
            }

            if(currentVal < target){
                leftFront.setPower(-speed);
                leftBack.setPower(-speed);
                rightFront.setPower(speed);
                rightBack.setPower(speed);
            }

            idle();
            currentVal = gyro.getIntegratedZValue();
        }

        stopDrive();
        idle();
    }

    public void loadAndReadyPlacer() throws InterruptedException{
        placer.setPosition(0);//load position
        sleep(400);
        placer.setPosition(128);//Ready position
    }

    public void loadShooter() throws InterruptedException{
        placer.setPosition(255);//shooter loading position
        sleep(300);
    }
}
