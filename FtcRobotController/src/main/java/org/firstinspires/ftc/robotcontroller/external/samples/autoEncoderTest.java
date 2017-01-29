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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@Autonomous(name="autoEncoderTest", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class autoEncoderTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;
    ModernRoboticsI2cGyro gyro = null;

    Servo selectionRight = null;
    Servo selectionLeft = null;
    ColorSensor beaconColorleft = null;
    ColorSensor beaconColorright = null;
    ColorSensor rightLineAlign = null;
    ColorSensor leftLineAlign = null;


    //OpticalDistanceSensor ods = null;
    OpticalDistanceSensor wallFollowing = null;
    DeviceInterfaceModule DIM = null;

    static final String ALLIANCE = "Blue";

    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double COUNTS_PER_MOTOR_REV_ANDYMARK = 1220;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_ANDYMARK = (COUNTS_PER_MOTOR_REV_ANDYMARK) / (WHEEL_DIAMETER_INCHES * 3.1415);
    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();

        waitForStart();
        runtime.reset();

        //DriveStraightGyro(50, 0.75);
        // EncoderDrive(0.75,10,'f');
        EncoderDrive(0.5, 5600, 7200);
        telemetry.addData("BeforeGyro: ", gyro.getIntegratedZValue());
        telemetry.update();

        telemetry.addData("AutoState: ", "First Selection Completed");
        telemetry.update();

        telemetry.addData("AutoState: ", "Second Selection Completed");
        telemetry.update();
        //EncoderDrive(0.85, 70);
        //EncoderDrive(-0.5, -20);

        telemetry.addData("Autonomous:", "Autonomous Complete!");


        while (opModeIsActive()) {
            String leftColor = getLeftBeaconColor();
            String rightColor = getRightBeaconColor();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("AfterGyro: " ,gyro.getIntegratedZValue());
            telemetry.addData("LightDetected: ", wallFollowing.getLightDetected());
            telemetry.addData("RawLightDetected: ", wallFollowing.getRawLightDetected());
            telemetry.addData("RawMax ", wallFollowing.getRawLightDetectedMax());
            telemetry.addData("LeftColor: " , leftColor);
            telemetry.addData("RightColor: ", rightColor);
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
    public void initializeRobot() throws InterruptedException{

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        //wallFollowing = hardwareMap.opticalDistanceSensor.get("wallFollowing");

        selectionLeft = hardwareMap.servo.get("selectionLeft");
        selectionRight = hardwareMap.servo.get("selectionRight");

        beaconColorleft = hardwareMap.colorSensor.get("beaconColorleft");
        beaconColorright = hardwareMap.colorSensor.get("beaconColorright");
        beaconColorleft.setI2cAddress(I2cAddr.create8bit(0x3c));
        beaconColorright.setI2cAddress(I2cAddr.create8bit(0x10));
        gyro.setI2cAddress(I2cAddr.create8bit(0x20));
        beaconColorleft.enableLed(false);
        beaconColorright.enableLed(false);
        selectionLeft.setPosition(255);
        selectionRight.setPosition(0);

        //ods = hardwareMap.opticalDistanceSensor.get("ods");
        resetEncoders();
        gyro.calibrate();
        while(gyro.isCalibrating()){
            telemetry.addData("Gyro:", "Gyro is Calibrating!");
            telemetry.update();
            sleep(50);
            //idle();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
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

    public void EncoderDrive(double speed, int inches, int inchesHitechnic) throws InterruptedException {
        resetEncoders();
        runWithEncoders();
        leftFront.setTargetPosition(inches);
        leftBack.setTargetPosition(inches);
        rightFront.setTargetPosition(inchesHitechnic);
        rightBack.setTargetPosition(inchesHitechnic);
        SetRun_To_Position();
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        while ((leftFront.isBusy() || leftBack.isBusy() || rightFront.isBusy() || rightBack.isBusy()  && opModeIsActive())) {
            idle();
            sleep(5);
        }

        stopDrive();
    }

    public void TurnAbsolute(int target, double speed) throws InterruptedException{
        runWithoutEncoders();
        int zAccumulated = gyro.getIntegratedZValue();
        while((Math.abs(zAccumulated - target) > 3)){
            double power = (zAccumulated - target) * .1 * speed;
            Range.clip(power, -0.5, 0.5);
            turn(power);
            zAccumulated = gyro.getIntegratedZValue();
            telemetry.addData("gyroVal:", zAccumulated);
            telemetry.addData("powerGyro: ", power);
            telemetry.update();
        }
        stopDrive();
        idle();
    }

    public void DriveStraightGyro(int duration, double power) throws InterruptedException{
        runWithEncoders();
        resetEncoders();
        duration = duration * (int)COUNTS_PER_INCH;
        double leftSpeed;
        double rightSpeed;
        double target = gyro.getIntegratedZValue();
        double startPosition = leftFront.getCurrentPosition();

        while((leftFront.getCurrentPosition() < duration + startPosition) && opModeIsActive()){
            int zAccumulated = gyro.getIntegratedZValue();
            leftSpeed = power + (zAccumulated - target)/100;
            rightSpeed = power - (zAccumulated - target)/ 100;
            leftSpeed = Range.clip(leftSpeed, -1,1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            leftFront.setPower(leftSpeed);
            leftBack.setPower(leftSpeed);
            rightFront.setPower(rightSpeed);
            rightBack.setPower(rightSpeed);

            idle();
        }
        stopDrive();
        idle();
    }
    public String getRightBeaconColor(){
        if(beaconColorright.red() > beaconColorright.blue() && beaconColorright.red() > beaconColorright.green()){
            return "Red";
        }
        if(beaconColorright.blue() > beaconColorright.red() && beaconColorright.blue() > beaconColorright.green()){
            return "Blue";
        }
        else{
            return "None";
        }
    }
    public String getLeftBeaconColor(){
        if(beaconColorleft.red() > beaconColorleft.blue() && beaconColorleft.red() > beaconColorleft.green()){
            return "Red";
        }
        if(beaconColorleft.blue() > beaconColorleft.red() && beaconColorleft.blue() > beaconColorleft.green()){
            return "Blue";
        }
        else{
            return "None";
        }
    }

    public void driveUntilBeacon(double power) throws InterruptedException{
        runWithoutEncoders();
        String leftColor = getLeftBeaconColor();
        String rightColor = getRightBeaconColor();
        while(!reachedBeacon(rightColor, leftColor)){
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
            leftColor = getLeftBeaconColor();
            rightColor = getRightBeaconColor();
            telemetry.addData("LeftColor: " , leftColor);
            telemetry.addData("RightColor: ", rightColor);
            telemetry.addData("Power:", leftBack.getPower());
            telemetry.update();
        }
        stopDrive();
    }
    public boolean reachedBeacon(String rightColor, String leftColor){
        if((rightColor.equals("Blue") && leftColor.equals("Red")) || (rightColor.equals("Red") && leftColor.equals("Blue"))){
            return true;
        }
        else{
            return false;
        }
    }
    public void select() throws InterruptedException {
        String leftVal = getLeftBeaconColor();
        String rightVal = getRightBeaconColor();
        telemetry.addData("In Select", "I AM SELECTING");

        while (!leftVal.equals(ALLIANCE) || !rightVal.equals(ALLIANCE) && opModeIsActive()) {
            if (leftVal.equals(ALLIANCE)) {
                selectionLeft.setPosition(0);//Fill in with selection position
                sleep(100);
                selectionLeft.setPosition(255);//Back to init
            } else if (rightVal.equals(ALLIANCE)) {
                selectionRight.setPosition(255);//Fill in selection position
                sleep(100);
                selectionRight.setPosition(0);//Back to init
            } else {
                selectionLeft.setPosition(0);//Fill in with selection position
                sleep(100);
                selectionLeft.setPosition(255);//Back to init
            }
            leftVal = getLeftBeaconColor();
            rightVal = getRightBeaconColor();
        }
        selectionLeft.setPosition(255);
        selectionRight.setPosition(0);
    }
    public void driveWithTime(double speed, long timeInMili) throws InterruptedException{
        leftBack.setPower(speed);
        leftFront.setPower(speed);
        rightBack.setPower(speed);
        rightFront.setPower(speed);
        sleep(timeInMili);
        stopDrive();
    }

    public void turn(double speed){//right is posotive
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(-speed);
        rightBack.setPower(-speed);
    }
    public void turnWithTime(double rightSpeed, double leftSpeed, long timeInMili) throws InterruptedException{
        rightFront.setPower(rightSpeed);
        rightBack.setPower(rightSpeed);
        leftFront.setPower(leftSpeed);
        leftBack.setPower(leftSpeed);
        sleep(timeInMili);
        stopDrive();
    }
    public void stopDrive(){
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    public void mecanumRight(double power, int time) throws InterruptedException{
        leftBack.setPower(-power);
        rightBack.setPower(-power);
        leftFront.setPower(power);
        rightFront.setPower(power);
        sleep(time);
        stopDrive();
    }

    //public char getFrontLineColor(){
    //  if(frontLineColor.)
    //}
}
