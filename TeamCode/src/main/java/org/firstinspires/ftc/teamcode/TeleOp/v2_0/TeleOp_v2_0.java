package org.firstinspires.ftc.teamcode.TeleOp.v2_0;

import org.firstinspires.ftc.teamcode.threadopmode.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.lang.*;

@Disabled
@TeleOp(group="TeleOp_v2", name="TeleOp_v2_0")
public class TeleOp_v2_0 extends ThreadOpMode {

    //Declaring drivetrain members
    public DcMotor fr = null;
    public DcMotor fl = null;
    public DcMotor bl = null;
    public DcMotor br = null;
    public DcMotor mr = null;
    public DcMotor ml = null;

    //Declaring IMU Members
    BNO055IMU imu;
    public Orientation angles;
    boolean strafing = false;
    double currentHeading = 0;
    double heading = 0;
    double differential = 0;
    double factor = 0;
    double grabPos = 0;

    //Declaring carousel members
    public DcMotor car;

    //Intake system members
    public DcMotor pivot = null;
    public DcMotor intake = null;

    //Box members
    public Servo boxhinge = null;
    public Servo boxpivot = null;

    //Cascade members
    public DcMotorEx cascade = null;

    //Sleeper SleeperFunction
    public int TIME_TO_WAIT;
    public long startTime;

    //Cascade position
    public long cascpos;
    public int playOnce = 1;

    //Arm
    public int ARMTIME;
    public long Time;

    //Hardware initialization code
    @Override
    public void mainInit() {

        //Telemetry Init
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Drivetrain hardware
        fr  = hardwareMap.get(DcMotor.class, "frontright");
        fl  = hardwareMap.get(DcMotor.class, "frontleft");
        bl  = hardwareMap.get(DcMotor.class, "backleft");
        br  = hardwareMap.get(DcMotor.class, "backright");

        //Reversing power for left wheels
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        //IMU Hardware
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Carousel hardware
        car = hardwareMap.get(DcMotor.class, "carousel");

        //Intake and box hardware
        pivot  = hardwareMap.get(DcMotor.class, "pivot");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        boxhinge = hardwareMap.get(Servo.class, "box");
        boxpivot = hardwareMap.get(Servo.class, "boxpivot");

        //Cascade hardware
        cascade = hardwareMap.get(DcMotorEx.class, "cascade");
        cascpos = 0;

        //Driver 1
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                if (y == 0.0) {
                    fl.setPower(x);
                    bl.setPower(-x);
                    fr.setPower(-x);
                    br.setPower(x);
                }

                if (x == 0.0) {
                    fl.setPower(y);
                    bl.setPower(y);
                    fr.setPower(y);
                    br.setPower(y);

                }

                if (Math.abs(rx) > 0) {
                    fl.setPower(rx);
                    bl.setPower(rx);
                    fr.setPower(-rx);
                    br.setPower(-rx);
                }
                //Wheel turning
                while (gamepad1.b) {
                    car.setPower(0.6);
                }

                while (gamepad1.x) {
                    car.setPower(-0.6);
                }

                car.setPower(0);

                //Intake arm turning
                while (gamepad1.y) {
                    pivot.setPower(0.6);
                }

                while (gamepad1.a) {
                    pivot.setPower(-0.6);
                }

                pivot.setPower(0);

                //Intake
                while (gamepad1.right_bumper) {
                    intake.setPower(0.9);
                }

                while (gamepad1.left_bumper) {
                    intake.setPower(-0.9);
                }

                intake.setPower(0);

                telemetry.update();

                if (gamepad1.dpad_up) {
                    pivot.setPower(1);

                    Time = System.currentTimeMillis();
                    while ((System.currentTimeMillis() - startTime) > ARMTIME) {
                        ARMTIME = 10000;

                    }

                }
                if (gamepad1.dpad_down) {

                    pivot.setPower(-0.9);

                    Time = System.currentTimeMillis();
                    while ((System.currentTimeMillis() - startTime) > ARMTIME) {
                        ARMTIME = 10000;

                    }
                }
            }
        }));

        //Driver 2
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cascade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                //Box Drop
                if (gamepad2.right_bumper) {
                    boxpivot.setPosition(1); //good for dropping
                    TIME_TO_WAIT = 500;
                    startTime = System.currentTimeMillis();
                    while ((System.currentTimeMillis() - startTime) < TIME_TO_WAIT) {

                    }
                    boxhinge.setPosition(0.7); //good for dropping
                }

                //Box Pickup
                if (gamepad2.left_bumper) {
                    boxpivot.setPosition(0.85); //good for rest
                    boxhinge.setPosition(1.4); //good for rest
                }

                //Cascade Ground
                if (gamepad2.a) {
                    cascade.setTargetPosition(-200);
                    cascade.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if (cascade.isBusy()) {
                    cascade.setPower(1500);
                } else {
                    cascade.setVelocity(0);
                }

                //Cascade Level 1
                if (gamepad2.x) {
                    cascade.setTargetPosition(400);
                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cascade.setVelocity(1500);
                }

                //Cascade Level 2
                while (gamepad2.b) {
                    cascade.setTargetPosition(-400);
                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cascade.setVelocity(1500);
                }

                //Cascade Level 3
                while (gamepad2.y) {
                    cascade.setTargetPosition(5000);
                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cascade.setVelocity(1500);
                }

            }
        }));

    }

    @Override
    public void mainLoop() {
        //No need
    }
}