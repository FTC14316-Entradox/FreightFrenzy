package org.firstinspires.ftc.teamcode.TeleOp.v1_0;

import org.firstinspires.ftc.teamcode.threadopmode.*;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group="TeleOp_v1", name="TeleOp_v1_2")
public class TeleOp_v1_2 extends ThreadOpMode {

    //Declaring drivetrain members
    public DcMotor fr = null;
    public DcMotor fl = null;
    public DcMotor bl = null;
    public DcMotor br = null;
    public DcMotor mr = null;
    public DcMotor ml = null;

    //Declaring arm members
    public DcMotor arm = null;
    public Servo lGrab = null;
    public Servo rGrab = null;

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
    public CRServo car;

    //Declaring cascading slide members
    public CRServo cascade1;
    public CRServo cascade2;
    public Servo dump = null;

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

        //Arm Hardware
        arm = hardwareMap.get(DcMotor.class, "arm");
        lGrab = hardwareMap.get(Servo.class, "lgrab");
        rGrab = hardwareMap.get(Servo.class, "rgrab");

        //Carousel hardware
        car = hardwareMap.get(CRServo.class, "carousel");

        //Cascading slide hardware
        cascade1 = hardwareMap.get(CRServo.class, "cascade1");
        cascade2 = hardwareMap.get(CRServo.class, "cascade2");

        //Dump hardware
        dump = hardwareMap.get(Servo.class, "dump");

        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                if (x>0) {
                    x = x*x;
                }
                if (x<0) {
                    x = -(x*x);
                }

                if (y>0) {
                    x = x*x;
                }
                if (y<0) {
                    x = -(x*x);
                }

                if (y == 0.0) {
                    strafing = true;
                    currentHeading = angles.firstAngle;
                    differential = heading-currentHeading;
                    factor = differential*0.005;

                    fl.setPower(x-factor);
                    bl.setPower(-(x-factor));
                    fr.setPower(-(x-factor));
                    br.setPower(x);
                }

                if (x == 0.0) {
                    strafing = false;
                    heading = angles.firstAngle;

                    fl.setPower(y);
                    bl.setPower(y);
                    fr.setPower(y);
                    br.setPower(y);
                    /* mr.setPower(y);
                    ml.setPower(y); */
                }

                if (Math.abs(rx) > 0) {
                    strafing = false;
                    heading = angles.firstAngle;

                    fl.setPower(rx);
                    bl.setPower(rx);
                    fr.setPower(-rx);
                    br.setPower(-rx);
                }

                if (gamepad1.x) {
                    dump.setPosition(0.25);
                }

                if (gamepad1.b) {
                    dump.setPosition(0.7);
                }

                telemetry.addData("Heading", heading);
                telemetry.addData("CurrentHeading", currentHeading);
                telemetry.addData("Differential", heading-currentHeading);

                telemetry.update();

            }
        }));

        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                if (gamepad2.right_trigger == 1.0) {
                    lGrab.setPosition(0.5);
                    rGrab.setPosition(0.1);

                }

                if (gamepad2.left_trigger == 1.0) {
                    lGrab.setPosition(0.4);
                    rGrab.setPosition(0.3);
                }

                while (gamepad2.y) {
                    arm.setPower(1.0);
                }
                while (gamepad2.a) {
                    arm.setPower(-1.0);
                }
                arm.setPower(0);

                //Wheel turning
                while (gamepad2.b) {
                    car.setDirection(DcMotorSimple.Direction.REVERSE);
                    car.setPower(-0.8);
                }

                while (gamepad2.x) {
                    car.setDirection(DcMotorSimple.Direction.FORWARD);
                    car.setPower(-0.8);
                }

                car.setPower(0);

                while (gamepad2.right_bumper) {
                    cascade1.setDirection(DcMotorSimple.Direction.FORWARD);
                    cascade2.setDirection(DcMotorSimple.Direction.FORWARD);
                    cascade1.setPower(-0.9);
                    cascade2.setPower(-0.9);
                }

                while (gamepad2.left_bumper) {
                    cascade1.setDirection(DcMotorSimple.Direction.REVERSE);
                    cascade2.setDirection(DcMotorSimple.Direction.REVERSE);
                    cascade1.setPower(-0.5);
                    cascade2.setPower(-0.5);
                }

                cascade1.setPower(0);
                cascade2.setPower(0);
            }
        }));

    }

    @Override
    public void mainLoop() {
        //Keep empty
    }
}
