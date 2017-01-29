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

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
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

@Autonomous(name="RobotAuto", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class RobotAuto extends RobotLinearHardwareParent {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double SHOOT_GEAR_REDUCTION = 1.0; //need to fill out
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_SHOOT = (COUNTS_PER_MOTOR_REV * SHOOT_GEAR_REDUCTION);
    int alliance = 2;
    int prevColor;
    DcMotor leftFrontmotor = null;
    DcMotor rightFrontmotor = null;
    DcMotor leftBackmotor = null;
    DcMotor rightBackmotor = null;
    ColorSensor beaconColor = null;
    Servo beaconPress = null;
    GyroSensor sensorGyro;
    ModernRoboticsI2cGyro gyro;

   // Servo beaconPress = null;
    //ColorSensor beaconColor = null;


    //need to reverse the motors

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontmotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontmotor = hardwareMap.dcMotor.get("rightFront");
        leftBackmotor = hardwareMap.dcMotor.get("leftBack");
        rightBackmotor = hardwareMap.dcMotor.get("rightBack");

        leftBackmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //beaconPress = hardwareMap.servo.get("beaconPress");
        //beaconColor = hardwareMap.colorSensor.get("beaconColor");
        //beaconColor.enableLed(false);

        // make sure the gyro is calibrated.

        resetEncoders();

        waitForStart();
        runtime.reset();

        encoderDriveforwards(0.75,1440 *2); //drive forwards so that we are positioned between the corner vortex and the beacon
        //Drive right, get encoder conversion









        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Start", "Autonomous Started");
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
    public void turn(int target) throws InterruptedException {
        turnAbsolute(target + gyro.getIntegratedZValue());
    }

    public void turnAbsolute(int target) throws InterruptedException {
        double zAccumalated = gyro.getIntegratedZValue();
        double turnSpeed = 0.15;

        while (Math.abs(zAccumalated - target) > 3) {

            if (zAccumalated > target) {
                leftFrontmotor.setPower(turnSpeed);
                rightFrontmotor.setPower(-turnSpeed);
                rightBackmotor.setPower(-turnSpeed);
                leftBackmotor.setPower(turnSpeed);
            }
            if (zAccumalated < target) {
                leftFrontmotor.setPower(-turnSpeed);
                rightFrontmotor.setPower(turnSpeed);
                rightBackmotor.setPower(turnSpeed);
                leftBackmotor.setPower(-turnSpeed);
            }


            idle();

            zAccumalated = gyro.getIntegratedZValue();

        }
        leftBackmotor.setPower(0);
        rightBackmotor.setPower(0);
        leftFrontmotor.setPower(0);
        rightFrontmotor.setPower(0);
        idle();

    }

    public void encoderDriveforwards(double speed,
                                     int inches) throws InterruptedException {
        leftFrontmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontmotor.setTargetPosition(inches);
        rightBackmotor.setTargetPosition(inches);


        rightBackmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontmotor.setPower(speed);
        rightBackmotor.setPower(speed);
        leftBackmotor.setPower(speed);
        rightFrontmotor.setPower(speed);

        telemetry.addData("Status:", "Entering Loop");
        while(leftFrontmotor.isBusy() || rightBackmotor.isBusy()){
            telemetry.addData("Status:", "In Loop");
            telemtry();
        }

        leftFrontmotor.setPower(0);
        rightFrontmotor.setPower(0);
        leftBackmotor.setPower(0);
        rightBackmotor.setPower(0);

        leftFrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*resetEncoders();
        int newTarget;

        if (opModeIsActive()) {


            newTarget = leftFrontmotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            leftFrontmotor.setTargetPosition(newTarget);
            rightBackmotor.setTargetPosition(newTarget);

            leftFrontmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftFrontmotor.setPower(speed);
            rightFrontmotor.setPower(speed);
            leftBackmotor.setPower(speed);
            rightBackmotor.setPower(speed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && (leftFrontmotor.isBusy() && rightFrontmotor.isBusy() && leftBackmotor.isBusy() && rightBackmotor.isBusy())) {
            }

            // Display it for the driver.
            leftFrontmotor.setPower(0);
            rightFrontmotor.setPower(0);
            leftBackmotor.setPower(0);
            rightBackmotor.setPower(0);

            telemetry.addData("EncoderVal", "theVal", leftBackmotor.getCurrentPosition(), rightFrontmotor.getCurrentPosition(), rightBackmotor.getCurrentPosition(), leftFrontmotor.getCurrentPosition());
            telemetry.update();
            sleep(250);
        }*/
    }

    public void driveRight(double speed,
                                     double inches) throws InterruptedException {
        resetEncoders();
        int newTarget;

        if (opModeIsActive()) {


            newTarget = leftFrontmotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            leftFrontmotor.setTargetPosition(newTarget);
            rightBackmotor.setTargetPosition(newTarget);

            leftFrontmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftFrontmotor.setPower(Math.abs(speed));
            rightFrontmotor.setPower(Math.abs(speed));
            leftBackmotor.setPower(Math.abs(speed));
            rightBackmotor.setPower(Math.abs(speed));

            leftFrontmotor.setPower(0);
            rightFrontmotor.setPower(0);
            leftBackmotor.setPower(0);
            rightBackmotor.setPower(0);

            sleep(250);
        }
    }

    public void moveBeacon(double speed) throws InterruptedException{
        while(opModeIsActive() && beaconColor() != 1 && beaconColor() != 2){
            leftFrontmotor.setPower(speed);
            rightFrontmotor.setPower(speed);
            leftBackmotor.setPower(speed);
            rightBackmotor.setPower(speed);
        }
        leftFrontmotor.setPower(0);
        rightFrontmotor.setPower(0);
        leftBackmotor.setPower(0);
        rightBackmotor.setPower(0);

        sleep(250);
    }

    public int beaconColor(){
        if(beaconColor.red() > beaconColor.blue() && beaconColor.red() > beaconColor.green()){
            return 1;
        }
        else if(beaconColor.blue() > beaconColor.red() && beaconColor.blue() > beaconColor.green()){
            return 2;
        }
        else{
            return 0;
        }
    }
    public void beaconUnequal(int nextColor, double speed) throws InterruptedException{
        while(opModeIsActive() && prevColor != nextColor){
            leftFrontmotor.setPower(speed);
            rightFrontmotor.setPower(speed);
            leftBackmotor.setPower(speed);
            rightBackmotor.setPower(speed);
        }
        leftFrontmotor.setPower(0);
        rightFrontmotor.setPower(0);
        leftBackmotor.setPower(0);
        rightBackmotor.setPower(0);
        sleep(250);

    }
    public void selectBeacon() throws InterruptedException{
        if(alliance == prevColor){
            beaconPress.setPosition(0);//Need to set position, select previous color
            wiggle();
        }
        else{
            beaconPress.setPosition(0);//Need to set position, select next color
            wiggle();
        }
    }
    public void wiggle() throws InterruptedException{
        for(int i = 0; i < 3; i++){
            leftFrontmotor.setPower(.1);
            rightFrontmotor.setPower(.1);
            leftBackmotor.setPower(.1);
            rightBackmotor.setPower(.1);
            sleep(250);
            leftFrontmotor.setPower(-.1);
            rightFrontmotor.setPower(-.1);
            leftBackmotor.setPower(-.1);
            rightBackmotor.setPower(-.1);
            sleep(500);
            leftFrontmotor.setPower(.1);
            rightFrontmotor.setPower(.1);
            leftBackmotor.setPower(.1);
            rightBackmotor.setPower(.1);
            sleep(250);
            leftFrontmotor.setPower(0);
            rightFrontmotor.setPower(0);
            leftBackmotor.setPower(0);
            rightBackmotor.setPower(0);
        }
    }

    public void resetEncoders() throws InterruptedException{

        leftFrontmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        leftFrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);;
        rightBackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void telemtry(){
        telemetry.addData("Run Time: " , runtime.toString());
        telemetry.addData("leftFrontEncoder" , leftFrontmotor.getCurrentPosition());
        telemetry.addData("rightBackEncoder" , rightBackmotor.getCurrentPosition());
        telemetry.addData("leftFront" , leftFrontmotor.getPower());
        telemetry.addData("leftBack" , leftBackmotor.getPower());
        telemetry.addData("rightFront" , rightFrontmotor.getPower());
        telemetry.addData("rightBack" , rightBackmotor.getPower());

    }

}