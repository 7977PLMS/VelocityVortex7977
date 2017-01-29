
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

@TeleOp(name="MotorTest", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class MotorTest extends OpMode
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

    double shooterInitVal = 0.4;

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
        shooterDetector = hardwareMap.opticalDistanceSensor.get("shooterDetector");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = hardwareMap.dcMotor.get("shooter");
        selection = hardwareMap.servo.get("selection");
        selection.setPosition(255);
        placer.setPosition(185);
        telemetry.addData("Status", "Initialized");


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
        if(gamepad1.a){
            leftFront.setPower(0.75);
        }
        else{
            leftFront.setPower(0);
        }
        if(gamepad1.b){
            leftBack.setPower(0.75);
        }
        else{
            leftBack.setPower(0);
        }
        if(gamepad1.x){
            rightFront.setPower(0.75);
        }
        else{
            rightFront.setPower(0);
        }
        if(gamepad1.y){
            rightBack.setPower(0.75);
        }
        else{
            rightBack.setPower(0);
        }

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


