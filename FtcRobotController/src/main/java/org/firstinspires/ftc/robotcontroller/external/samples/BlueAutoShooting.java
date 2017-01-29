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
import com.qualcomm.hardware.modernrobotics.ReadWriteRunnable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.nio.channels.GatheringByteChannel;

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

@Autonomous(name="Autonomous Shooting", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class BlueAutoShooting extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;

    Servo placer = null;
    DcMotor feeder = null;
    DcMotor shooter = null;
    OpticalDistanceSensor shooterDetector = null;

    Servo selection;
    ModernRoboticsI2cGyro gyro = null;

    String majorStep = " ";
    String minorStep = " ";


    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    double shooterInitVal = 0.15;




    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();

        waitForStart();
        runtime.reset();

        EncoderDrive(0.5, 40,40);
        turnAbsolute(-90, 0.5, 8);
        shoot();
        turnAbsolute(0, 0.5, 8);
        sleep(10000);
        EncoderDrive(0.5, 35, 35);

        while(opModeIsActive()){
            // addTelemetry();
        }
    }

    public void initializeRobot() throws InterruptedException {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        placer = hardwareMap.servo.get("placer");
        feeder = hardwareMap.dcMotor.get("feeder");
        shooter = hardwareMap.dcMotor.get("shooter");
        selection = hardwareMap.servo.get("selection");
        shooterDetector = hardwareMap.opticalDistanceSensor.get("shooterDetector");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.setI2cAddress(I2cAddr.create8bit(0x2u  0));;

        placer.setPosition(185);
        selectionOut();

        gyro.calibrate();
        while(gyro.isCalibrating()){
            sleep(5);
            telemetry.addData("Gyro: ", "Is Calibrating");
            telemetry.update();
            idle();
        }
        telemetry.addData("Gyro: ", "Isn't Calibrating");
        telemetry.update();

    }

    public void SetRun_To_Position() {
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runWithoutEncoders() {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runWithEncoders() {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void EncoderDrive(double speed, double inchesRight, double inchesLeft) throws InterruptedException {
        runWithEncoders();
        int targetRight = (int) (inchesRight *  COUNTS_PER_INCH);
        int targetLeft = (int) (inchesLeft *  COUNTS_PER_INCH);
        leftFront.setTargetPosition(targetLeft + leftFront.getCurrentPosition());
        rightFront.setTargetPosition(targetRight + rightFront.getCurrentPosition());
        leftBack.setTargetPosition(targetLeft + leftBack.getCurrentPosition());
        rightBack.setTargetPosition(targetRight + rightBack.getCurrentPosition());
        SetRun_To_Position();
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
        leftBack.setPower(speed);
        minorStep = "SetPosition";
        addTelemetry();
        while ((leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) && opModeIsActive()) {
            idle();
            sleep(5);
            minorStep = "InDriveLoop";
            addTelemetry();
        }
        minorStep = "StopDrive";
        addTelemetry();
        stopDrive();

    }
    public void turnWithEncoder(double speed, int inchesRight, int inchesLeft) throws InterruptedException {
        runWithEncoders();
        int targetRight = inchesRight * (int) COUNTS_PER_INCH;
        int targetLeft = inchesLeft * (int) COUNTS_PER_INCH;
        leftFront.setTargetPosition(targetLeft + leftFront.getCurrentPosition());
        rightFront.setTargetPosition(targetRight + rightFront.getCurrentPosition());
        leftBack.setTargetPosition(targetLeft + leftBack.getCurrentPosition());
        rightBack.setTargetPosition(targetRight + rightBack.getCurrentPosition());
        SetRun_To_Position();
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
        leftBack.setPower(speed);
        minorStep = "SetPosition";
        addTelemetry();
        while ((leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) && opModeIsActive()) {
            idle();
            sleep(5);
            minorStep = "InDriveLoop";
            addTelemetry();
        }
        minorStep = "StopDrive";
        addTelemetry();
        stopDrive();

    }

    public void stopDrive(){
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
    }
    public void turnAbsolute(int target, double speed, int buffer) throws InterruptedException{
        runWithoutEncoders();
        int currentVal = gyro.getIntegratedZValue();
        while(Math.abs(currentVal - target) > buffer){
            minorStep = "TurningToPos";
            addTelemetry();
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
        minorStep = "StopDrive";
        stopDrive();
        idle();
    }

    public void placeBall() throws InterruptedException{
        placer.setPosition(0);//Place ball in shooter
        sleep(1000);
        placer.setPosition(185);//Bring placer back
    }
    public void waitUntilShot(){
        while(shooterDetector.getLightDetected() > shooterInitVal){}
    }
    public void shoot() throws InterruptedException{
        shooter.setPower(0.75);
        minorStep = "ShooterOn";
        addTelemetry();
        waitUntilShot();
        sleep(500);
        placeBall();
        addTelemetry();
        waitUntilShot();
        sleep(500);
        shooter.setPower(0);
        addTelemetry();

    }
    public void selectionOut(){
        selection.setPosition(0);
    }


    public void addTelemetry(){
        telemetry.addData("MajorStep: ", majorStep);
        telemetry.addData("MinorStep: ", minorStep);
        telemetry.addData("ShooterDetector: ", shooterDetector.getLightDetected());
        telemetry.update();
    }


}

