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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

@Autonomous(name="RobotAutoTime", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class RobotAutoTime extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;
    ColorSensor beaconColor;
    //DeviceInterfaceModule dvim;
    //ColorSensor beaconColor = null;
    Servo beaconPress = null;
    //Servo touchServo = null;
    // TouchSensor touchWall = null;
   // ModernRoboticsI2cGyro gyro = null;
    float initPosBeacon = 1.0f;
    float halfPosBeacon = 0.6f;
    float fullPosBeacon = 0.0f;
    float outPosTouch = 0.0f;
    float inPosTouch = 0.0f;
    static final int ALLIANCE = 2;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() throws InterruptedException {
        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        beaconColor = hardwareMap.colorSensor.get("color");
        //dvim = hardwareMap.deviceInterfaceModule.get("dvim");
        beaconPress = hardwareMap.servo.get("beaconPress");


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        beaconColor.enableLed(false);
        beaconPress.setPosition(0);//Init Position

        waitForStart();
        runtime.reset();

        encoderDriveforwards(0.6, 35); //FILL IN PROPER INFORMATION FOR MOVING 40 IN
        encoderDriveLeft(0.6, 23);
        encoderDriveforwards(0.6, 54);
        encoderDriveRight(0.6, 23);
        encoderDriveforwards(0.6, 7);
        mecanumEncoderRight(20, 0.3);
        mecanumEncoderRightBack(20, 0.3);
        telemetry.addData("Progress: ", "before telemetry");
        telemetry.update();
        sleep(100);
        driveToBeacon(0.5);
        int prevSelectColor = beaconColor();
        beaconUnequal(0, 0.3);
        if(ALLIANCE == prevSelectColor){
            prevSelection(prevSelectColor, 0.3);
        }
        else{
            nextSelection(ALLIANCE, 0.3);
        }


        encoderDriveforwards(0.5, 20);


        driveToBeacon(0.5);
        prevSelectColor = beaconColor();
        beaconUnequal(0, 0.3);
        if(ALLIANCE == prevSelectColor){
            prevSelection(prevSelectColor, 0.3);
        }
        else{
            nextSelection(ALLIANCE, 0.3);
        }






        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();




            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }


    public void forwardTime(double speed, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            leftFront.setPower(speed);
            rightFront.setPower(speed);
            leftBack.setPower(speed);
            rightBack.setPower(speed);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void encoderDriveforwards(double speed,
                                     int inches) throws InterruptedException {

        int target = inches * (int) COUNTS_PER_INCH;
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        leftBack.setTargetPosition(target);
        rightBack.setTargetPosition(target);

        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        rightBack.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {

        }

        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);


    }
    public void encoderDriveRight(double speed,
                                     int inches) throws InterruptedException {

        int target = inches * (int) COUNTS_PER_INCH;
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setTargetPosition(-target);
        rightFront.setTargetPosition(target);
        leftBack.setTargetPosition(-target);
        rightBack.setTargetPosition(target);

        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(-speed);
        rightBack.setPower(speed);
        leftBack.setPower(-speed);
        rightFront.setPower(speed);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {

        }

        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);


    }
    public void encoderDriveLeft(double speed,
                                  int inches) throws InterruptedException {

        int target = inches * (int) COUNTS_PER_INCH;
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setTargetPosition(target);
        rightFront.setTargetPosition(-target);
        leftBack.setTargetPosition(target);
        rightBack.setTargetPosition(-target);

        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        rightBack.setPower(-speed);
        leftBack.setPower(speed);
        rightFront.setPower(-speed);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {

        }

        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);


    }

    public void turn(double time, double leftSpeed, double rightSpeed) throws InterruptedException {
        runtime.reset();
        while(runtime.seconds() < time){
            leftFront.setPower(leftSpeed);
            leftBack.setPower(leftSpeed);
            rightFront.setPower(rightSpeed);
            rightBack.setPower(rightSpeed);
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
       /* int targetRight = rightInches * (int) COUNTS_PER_INCH;
        int targetLeft = leftInches * (int) COUNTS_PER_INCH;
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setTargetPosition(targetLeft);
        rightFront.setTargetPosition(targetRight);
        leftBack.setTargetPosition(targetLeft);
        rightBack.setTargetPosition(targetRight);

        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        rightBack.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {

        }

        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);

        sleep(250);*/
    }
    public void mecanumEncoderRight(int inches, double speed){
        int target = inches * (int) COUNTS_PER_INCH;
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setTargetPosition(target);
        rightFront.setTargetPosition(-target);
        leftBack.setTargetPosition(-target);
        rightBack.setTargetPosition(target);

        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        rightBack.setPower(speed);
        leftBack.setPower(-speed);
        rightFront.setPower(-speed);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {

        }

        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
    }
    public void mecanumEncoderRightBack(int inches, double speed){
        int target = inches * (int) COUNTS_PER_INCH;
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setTargetPosition(-target);
        rightBack.setTargetPosition(target);

        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        rightBack.setPower(speed);
        leftBack.setPower(-speed);

        while (leftBack.isBusy() && rightBack.isBusy()) {

        }

        rightBack.setPower(0);
        leftBack.setPower(0);
    }
    public int beaconColor(){
        if(beaconColor.red() > beaconColor.blue() && beaconColor.red() > beaconColor.green()){
        //    dvim.setLED(1, true);
         //   dvim.setLED(0, false);
            return 1;

        }
        else if(beaconColor.blue() > beaconColor.red() && beaconColor.blue() > beaconColor.green()){
        //    dvim.setLED(1, false);
         //   dvim.setLED(0, true);
            return 2;
        }
        else{
           // dvim.setLED(1, false);
            //dvim.setLED(0, false);
            return 0;
        }
    }
    public void beaconUnequal(int nextColor, double speed) throws InterruptedException{
        int prevColor = beaconColor();
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
        while(opModeIsActive() && prevColor != nextColor){
            prevColor = beaconColor();
            sleep(5);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(250);

    }
    public void driveToBeacon( double speed) throws InterruptedException{
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("To Beacon", "Entering");
        telemetry.update();
        int prevColor = beaconColor();
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
        while(opModeIsActive()&& prevColor == 0){
            telemetry.addData("To Beacon:", "In");
            prevColor = beaconColor();
            telemetry.addData("Color:", prevColor);
            telemetry.update();
            idle();
            sleep(5);
        }
        telemetry.addData("To Beacon:", "Out");
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(250);
        telemetry.update();
    }
    public void encoderDriveback(double speed,
                                     int inches) throws InterruptedException {

        int target = inches * (int) COUNTS_PER_INCH;
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setTargetPosition(-target);
        rightFront.setTargetPosition(-target);
        leftBack.setTargetPosition(-target);
        rightBack.setTargetPosition(-target);

        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(-speed);
        rightBack.setPower(-speed);
        leftBack.setPower(-speed);
        rightFront.setPower(-speed);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {

        }

        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);


    }

    public void prevSelection(int prevColor, double speed) throws InterruptedException{
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentColor = beaconColor();
        leftBack.setPower(-speed);
        rightBack.setPower(-speed);
        leftFront.setPower(-speed);
        rightFront.setPower(-speed);
        while(currentColor != prevColor){
            currentColor = beaconColor();
        }
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        beaconPress.setPosition(0.5);
        sleep(200);
        mecanumEncoderRight(5, 0.4);
        beaconPress.setPosition(0.6);//fill in value for pressing
        sleep(200);
        mecanumEncoderLeft(5, 0.3);
        beaconPress.setPosition(0);
        mecanumEncoderRight(5, 0.4);
        beaconPress.setPosition(0.6);
        mecanumEncoderLeft(5, 0.3);
        beaconPress.setPosition(0);
        mecanumEncoderRight(5, 0.4);
        beaconPress.setPosition(0.6);

    }
    public void nextSelection(int prevColor, double speed) throws InterruptedException{
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentColor = beaconColor();
        leftBack.setPower(speed);
        rightBack.setPower(speed);
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        while (currentColor != prevColor){
            currentColor = beaconColor();
        }
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        beaconPress.setPosition(0.5);
        sleep(200);
        mecanumEncoderRight(5, 0.4);
        beaconPress.setPosition(0.6);//fill in value for pressing
        sleep(200);
        mecanumEncoderLeft(5, 0.6);
        beaconPress.setPosition(0);
        mecanumEncoderRight(5, 0.4);
        beaconPress.setPosition(0.6);
        mecanumEncoderLeft(5, 0.3);
        beaconPress.setPosition(0);
        mecanumEncoderRight(5, 0.4);
        beaconPress.setPosition(0.6);
    }

    public void mecanumEncoderLeft(int inches, double speed){
        int target = inches * (int) COUNTS_PER_INCH;
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setTargetPosition(-target);
        rightFront.setTargetPosition(target);
        leftBack.setTargetPosition(target);
        rightBack.setTargetPosition(-target);

        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(-speed);
        rightBack.setPower(-speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {

        }

        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
    }



}


