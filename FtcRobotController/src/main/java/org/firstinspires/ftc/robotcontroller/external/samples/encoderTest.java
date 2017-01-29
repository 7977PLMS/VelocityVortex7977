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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@Autonomous(name="encoderTest", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class encoderTest extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    double target;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        encoderForward(0.5, 24, 5);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemtry();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public void encoderForward(double speed, double distance, double timeOut){
        int leftFrontpos, rightBackpos;
        target = distance * COUNTS_PER_INCH;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition((int) target);
        rightBack.setTargetPosition((int) target);
        leftFront.setPower(speed);
        rightBack.setPower(speed);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        telemetry.addData("Loop:", "Starting Loop");


       /* while(runtime.seconds() < timeOut){
            leftFrontpos = leftFront.getCurrentPosition();
            rightBackpos = rightBack.getCurrentPosition();

            if(leftFrontpos < target){
                leftFront.setPower(speed);
                leftBack.setPower(speed);
            }
            else{
                leftFront.setPower(0);
                leftBack.setPower(0);
            }
            if(rightBackpos < target){
                rightFront.setPower(speed);
                rightBack.setPower(speed);
            }
            else{
                rightFront.setPower(0);
                rightBack.setPower(0);
            }

            telemtry();

        }*/


        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);

        telemtry();
        telemetry.addData("target", "targetReached");

    }

    public void telemtry(){
        DbgLog.msg( "Run Time: " + runtime.toString());
        DbgLog.msg("leftFrontEncoder" + leftFront.getCurrentPosition());
        DbgLog.msg("rightBackEncoder" + rightBack.getCurrentPosition());
        DbgLog.msg("leftFront" + leftFront.getPower());
        DbgLog.msg("leftBack" + leftBack.getPower());
        DbgLog.msg("rightFront" + rightFront.getPower());
        DbgLog.msg("rightBack" + rightBack.getPower());
        telemetry.addData("Target", target);
    }
}
