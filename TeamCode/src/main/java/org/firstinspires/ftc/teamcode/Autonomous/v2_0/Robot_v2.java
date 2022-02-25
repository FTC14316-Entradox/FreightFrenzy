package org.firstinspires.ftc.teamcode.Autonomous.v2_0;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot_v2 {

    public DcMotor fr;
    public DcMotor fl;
    public DcMotor bl;
    public DcMotor br;

    public DcMotor car;
    public DcMotor pivot;
    public DcMotor intake;
    public Servo boxhinge;
    public Servo boxpivot;
    public DcMotorEx cascade;

    float Kp = 0;
    float Ki = 0;
    float Kd = 0;

    float errorFront = 0;
    float errorBack = 0;
    float lastErrorF = 0;
    float lastErrorB = 0;
    float integralFront = 0;
    float integralBack = 0;
    float derivativeF = 0;
    float derivativeB = 0;

    long time;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    float slideKp = 0;
    double slideError;
    int target;

    /* ------------------------------------ General ---------------------------------------------- */

    public Robot_v2(HardwareMap hardwareMap) {
        car = hardwareMap.get(DcMotor.class, "carousel");
        pivot  = hardwareMap.get(DcMotor.class, "pivot");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        boxhinge = hardwareMap.get(Servo.class, "box");
        boxpivot = hardwareMap.get(Servo.class, "boxpivot");
        cascade = hardwareMap.get(DcMotorEx.class, "cascade");
        fr  = hardwareMap.get(DcMotor.class, "frontright");
        fl  = hardwareMap.get(DcMotor.class, "frontleft");
        bl  = hardwareMap.get(DcMotor.class, "backleft");
        br  = hardwareMap.get(DcMotor.class, "backright");
        cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public static void pause(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            System.err.format("IOException: %s%n", e);
        }
    }

    public void setCascade(int position) {
        double currentPos = cascade.getCurrentPosition();

        if (position == 3) { target = 8000; }
        if (position == 2) { target = 4000; }
        if (position == 1) { target = 1000; }
        if (position == 0) { target = 0; }

        cascade.setTargetPosition(target);
        cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (true) {
            slideError = target - cascade.getCurrentPosition();
            cascade.setVelocity(slideError * slideKp);
        }
    }

    /* ------------------------------------ TeleOp Specific ------------------------------------------- */

    public void carousel(double power) { car.setPower(power); }

    public void safeBox() {
        boxpivot.setPosition(0.12);
        boxhinge.setPosition(0.2);
    }

    public void dropBox() {
        boxpivot.setPosition(0.5);
        boxhinge.setPosition(0.2);
    }

    /* ------------------------------------ Autonomous Specific ------------------------------------------- */

    public void carouselTime(double power, int milliseconds) {
        car.setPower(power);
        pause(milliseconds);
        car.setPower(0);
    }

    public void forward(double power, double seconds) {
        time = System.currentTimeMillis();

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

        while (seconds < (System.currentTimeMillis() - time)) {
            errorFront = fl.getCurrentPosition() - fr.getCurrentPosition();
            errorBack = bl.getCurrentPosition() - br.getCurrentPosition();
            integralFront = integralFront + errorFront;
            integralBack = integralBack + errorBack;
            derivativeF = errorFront - lastErrorF;
            derivativeB = errorBack - lastErrorB;

            fr.setPower(fl.getPower() + (errorFront * Kp) + (integralFront * Ki) + (derivativeF * Kd));
            br.setPower(bl.getPower() + (errorBack * Kp) + (integralBack * Ki) + (derivativeB * Kd));

            lastErrorB = errorBack;
            lastErrorF = errorFront;

            pause(15);
        }

        brake();
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    public void strafe(double power, double seconds) {
        time = System.currentTimeMillis();

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(power);

        while (seconds < (System.currentTimeMillis() - time)) {
            errorFront = fl.getCurrentPosition() - fr.getCurrentPosition();
            errorBack = bl.getCurrentPosition() - br.getCurrentPosition();
            integralFront = integralFront + errorFront;
            integralBack = integralBack + errorBack;
            derivativeF = errorFront - lastErrorF;
            derivativeB = errorBack - lastErrorB;

            fr.setPower(-(fl.getPower() + (errorFront * Kp) + (integralFront * Ki) + (derivativeF * Kd)));
            br.setPower(-(bl.getPower() + (errorBack * Kp) + (integralBack * Ki) + (derivativeB * Kd)));

            lastErrorB = errorBack;
            lastErrorF = errorFront;

            pause(15);
        }

        brake();
    }

    public void brake() {
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

}