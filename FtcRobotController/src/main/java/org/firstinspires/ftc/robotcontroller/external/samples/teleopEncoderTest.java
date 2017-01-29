
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.InputMismatchException;

@TeleOp(name="EncoderTestTeleop", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class teleopEncoderTest extends OpMode
{
    // Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    double tempSpeed = 0.75;
    double slowSpeed = 0.25;

    DcMotor leftFrontmotor = null;
    DcMotor rightFrontmotor = null;
    DcMotor leftBackmotor = null;
    DcMotor rightBackmotor = null;
    DcMotor feeder1 = null;
    DcMotor feeder2 = null;
    DcMotor shooter = null;
    Servo gate = null;
    OpticalDistanceSensor ods = null;


    public void MecanumDrive()
    {
        DbgLog.msg("Testtest");
        //throw new InputMismatchException();

        //leftFrontmotor = hardwareMap.dcMotor.get("1");
        //rightFrontmotor = hardwareMap.dcMotor.get("2");
        //leftBackmotor = hardwareMap.dcMotor.get("3");
        //rightBackmotor = hardwareMap.dcMotor.get("4");

    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()  {
        map(hardwareMap);
        //throw new InputMismatchException();
        telemetry.addData("Status", "Initialized");
        //      leftFrontmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //    leftBackmotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void init_loop() {
        //telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() { runtime.reset();
    }

    @Override
    public void loop(){

        telemetry.addData("Status", "Test1");
        float tankRight = -gamepad1.right_stick_y;
        float tankLeft = -gamepad1.left_stick_y;


        //Still need to create reaction threshold, can do so once read the actual raw values
        telemetry.addData("Status", "%.2f", tankLeft);
        telemetry.addData("Status", "%.2f", tankRight);



        if (leftFrontmotor == null) {
            telemetry.addData("Status", "leftFront is hosed");
        }
        //telemetry.addData("Status", leftBackmotor.getPower());

        if(gamepad1.a){
            double speed = 0.75d;

            rightBackmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackmotor.setTargetPosition(2880);
            rightBackmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackmotor.setPower(speed);

            while (rightBackmotor.isBusy()) {
            }

            rightBackmotor.setPower(0);

        }
        if(gamepad1.b){
            double speed = 0.75d;

            leftBackmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackmotor.setTargetPosition(2880);
            leftBackmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackmotor.setPower(speed);

            while (leftBackmotor.isBusy()) {
            }

            leftBackmotor.setPower(0);
        }
        if(gamepad1.x){
            double speed = 0.75d;

            rightFrontmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontmotor.setTargetPosition(2880);
            rightFrontmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontmotor.setPower(speed);

            while (rightFrontmotor.isBusy()) {
            }

            rightFrontmotor.setPower(0);
        }
        if(gamepad1.y){
            double speed = 0.75d;

            leftFrontmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontmotor.setTargetPosition(2880);
            leftFrontmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontmotor.setPower(speed);

            while (leftFrontmotor.isBusy()) {
            }

            leftFrontmotor.setPower(0);
        }


        telemtry();

    }

    @Override
    public void stop() {
    }

    public void map(HardwareMap hwMap){

        leftFrontmotor= hwMap.dcMotor.get("leftFront");
        leftBackmotor  = hwMap.dcMotor.get("leftBack");
        rightBackmotor    = hwMap.dcMotor.get("rightFront");
        rightFrontmotor = hwMap.dcMotor.get("rightBack");

        //ods = hwMap.opticalDistanceSensor.get("ods");

        leftBackmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontmotor.setDirection(DcMotorSimple.Direction.REVERSE);


        //shooter.setDirection((DcMotorSimple.Direction.REVERSE));

    }
    public void telemtry(){
        telemetry.addData("FrontRight", "%.2f", rightFrontmotor.getPower());
        telemetry.addData("FrontLeft", "%.2f", leftFrontmotor.getPower());
        telemetry.addData("BackRight", "%.2f", rightBackmotor.getPower());
        telemetry.addData("BackLeft", "%.2f", leftBackmotor.getPower());
        telemetry.addData("currentTime", "%.2f", runtime.seconds());
    }
}
