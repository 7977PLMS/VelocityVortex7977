
package org.firstinspires.ftc.robotcontroller.external.samples;

import android.view.ViewDebug;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.InputMismatchException;

@TeleOp(name="MecanumDrive", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class MecanumDrive extends OpMode
{
    // Declare OpMode members. */
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;

    DcMotor shooter = null;
    DcMotor feeder = null;
    Servo placer = null;
    OpticalDistanceSensor shooterDetector = null;




    ColorSensor colorAlign = null;
    ColorSensor beaconColor = null;
    
    Servo selection = null;

    double mecanumSpeed = 0.8;
    enum State {HIGH, LOW};
    enum FeederDirection {FORWARD, REVERSE}

    State driveState = State.HIGH;
    State placerState = State.LOW;
    FeederDirection feederDirection = FeederDirection.FORWARD;
    State feederState = State.LOW;
    State beaconPressState = State.HIGH;

    double driveSwitchTime = 0;
    double feederSwitchTime = 0;
    double feederSwitchDirectionTime = 0;
    double beaconSwitchTime = 0;
    double runtimeTemp;

    double shooterInitVal = 0.15;

    private ElapsedTime runtime = new ElapsedTime();



    public void MecanumDrive()
    {
        DbgLog.msg("Testtest");

    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()  {

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

        colorAlign = hardwareMap.colorSensor.get("colorAlign");
        colorAlign.setI2cAddress(I2cAddr.create8bit(0x30));
        beaconColor = hardwareMap.colorSensor.get("beaconColor");
        beaconColor.setI2cAddress(I2cAddr.create8bit(0x3c));


        colorAlign.enableLed(true);
        beaconColor.enableLed(false);

        placer.setPosition(185);
        selection.setPosition(0);



    }

    @Override
    public void init_loop() {
        //telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
         float tankRight = -gamepad1.right_stick_y;
        float tankLeft = -gamepad1.left_stick_y;


        if (driveState == State.HIGH) {
            mecanumSpeed = 0.8;
        } else {
            mecanumSpeed = 0.4;
        }

        if (feederState == State.HIGH) {
            if (feederDirection == FeederDirection.REVERSE) {
                feeder.setPower(-0.70);
            } else {
                feeder.setPower(0.75);
            }
        } else {
            feeder.setPower(0);
        }

        if (placerState == State.HIGH) {
            placer.setPosition(0);//Placing particle
        } else {
            placer.setPosition(185);//Loading Particle
        }

        if (beaconPressState == State.HIGH) {
            selection.setPosition(0);
        } else {
            selection.setPosition(255);
        }

        if (gamepad1.dpad_up) {
            leftBack.setPower(mecanumSpeed);
            rightBack.setPower(mecanumSpeed);
            leftFront.setPower(mecanumSpeed);
            rightFront.setPower(mecanumSpeed);
        } else if (gamepad1.dpad_down) {
            leftBack.setPower(-mecanumSpeed);
            rightBack.setPower(-mecanumSpeed);
            leftFront.setPower(-mecanumSpeed);
            rightFront.setPower(-mecanumSpeed);
        } else if (gamepad1.dpad_left) {
            leftBack.setPower(mecanumSpeed);
            rightBack.setPower(-mecanumSpeed);
            leftFront.setPower(-mecanumSpeed);
            rightFront.setPower(mecanumSpeed);
        } else if (gamepad1.dpad_right) {
            leftBack.setPower(-mecanumSpeed);
            rightBack.setPower(mecanumSpeed);
            leftFront.setPower(mecanumSpeed);
            rightFront.setPower(-mecanumSpeed);

        } else {

            leftBack.setPower(tankLeft);
            rightFront.setPower(tankRight);
            rightBack.setPower(tankRight);
            leftFront.setPower(tankLeft);
        }

        if (gamepad1.x && driveState == State.HIGH && (runtime.time() - driveSwitchTime) > 0.5) {
            driveState = State.LOW;
            driveSwitchTime = runtime.time();
        } else if (gamepad1.x && driveState == State.LOW && (runtime.time() - driveSwitchTime) > 0.5) {
            driveState = State.HIGH;
            driveSwitchTime = runtime.time();
        }

        if (gamepad1.a && feederState == State.LOW && (runtime.time() - feederSwitchTime) > 0.5) {
            feederState = State.HIGH;
            feederSwitchTime = runtime.time();
        } else if (gamepad1.a && feederState == State.HIGH && (runtime.time() - feederSwitchTime) > 0.5) {
            feederState = State.LOW;
            feederSwitchTime = runtime.time();
        }

        if (gamepad1.y && feederDirection == FeederDirection.FORWARD && (runtime.time() - feederSwitchDirectionTime) > 0.5) {
            feederDirection = FeederDirection.REVERSE;
            feederSwitchDirectionTime = runtime.time();
        } else if (gamepad1.y && feederDirection == FeederDirection.REVERSE && (runtime.time() - feederSwitchDirectionTime) > 0.5) {
            feederDirection = FeederDirection.FORWARD;
            feederSwitchDirectionTime = runtime.time();
        }

        if (gamepad1.right_bumper) {
            placerState = State.HIGH;
        } else if (gamepad1.left_bumper) {
            placerState = State.LOW;
        }

        if (gamepad1.b && beaconPressState == State.HIGH && (runtime.time() - beaconSwitchTime) > 0.5) {
            beaconPressState = State.LOW;
            beaconSwitchTime = runtime.time();
        } else if (gamepad1.b && beaconPressState == State.LOW && (runtime.time() - beaconSwitchTime) > 0.5) {
            beaconPressState = State.HIGH;
            beaconSwitchTime = runtime.time();
        }

        if (gamepad1.right_trigger > 0.3) {
            feederState = State.HIGH;
            shooter.setPower(0.75);
            if (shooterDetector.getLightDetected() < shooterInitVal) {
                waitTime(0.5);
                placeBall();
            }
        } else if (gamepad1.left_trigger > 0.3) {
            shooter.setPower(0.75);
        } else {
            shooter.setPower(0);
        }
        addTelemtry();
    }

    @Override
    public void stop() {
    }

    public void addTelemtry(){
        telemetry.addData("FrontRight: ", "%.2f", rightFront.getPower());
        telemetry.addData("FrontLeft: ", "%.2f", leftFront.getPower());
        telemetry.addData("BackRight: ", "%.2f", rightBack.getPower());
        telemetry.addData("BackLeft: ", "%.2f", leftBack.getPower());
        telemetry.addData("Shooter: ", "%.2f", shooter.getPower());
        telemetry.addData("Feeder: ", "%.2f", feeder.getPower());
        telemetry.addData("currentTime", "%.2f", runtime.seconds());
        telemetry.addData("RawVal: ", shooterDetector.getRawLightDetected());
        telemetry.addData("LightVal: ", shooterDetector.getLightDetected());
        telemetry.addData("ColorlineAlpha: ", colorAlign.alpha());
        telemetry.addData("ColorlineArgb: ", colorAlign.argb());

        telemetry.addData("ColorlineRed: ", colorAlign.red());

        telemetry.addData("ColorlineBlue: ", colorAlign.blue());

        telemetry.addData("ColorlineGreen: ", colorAlign.green());
    }
    public void stopDrive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void placeBall(){
        placer.setPosition(0);//Place position
        placerState = State.HIGH;
        waitTime(0.75);
        placer.setPosition(185);//Placer load
        placerState = State.LOW;
    }

    public void waitTime(double time){
        runtimeTemp = runtime.time();
        while((runtime.time() - runtimeTemp) < time){}
    }





}
