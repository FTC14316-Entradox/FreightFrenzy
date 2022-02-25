package org.firstinspires.ftc.teamcode.Autonomous.v2_0;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot {

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

    public Robot(HardwareMap hardwareMap) {
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

    public void carousel(double power) { car.setPower(power); }

    public void carouselTime(double power, int milliseconds) {
        car.setPower(power);
        pause(milliseconds);
        car.setPower(0);
    }

    public void drive(double power, int heading) {
        if (heading == 1){ //go forwards or backwards
            fr.setPower(power);
            fl.setPower(power);
            br.setPower(power);
            bl.setPower(power);
        }
        if (heading == 2) { //go right or left
            fr.setPower(-power);
            fl.setPower(power);
            br.setPower(power);
            bl.setPower(-power);
        }
    }

    public void stop() {
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    public void safeBox() {
        boxpivot.setPosition(0.12);
        boxhinge.setPosition(0.2);
    }

    public void parallelBox() {
        boxpivot.setPosition(0.25);
        boxhinge.setPosition(0.0);
    }

    public void goToLvl(int level) {
        if (level == 1) {
            cascade.setTargetPosition(180);
        }
        if (level == 2) {
            cascade.setTargetPosition(1000);
        }
        if (level == 3) {
            cascade.setTargetPosition(8000);
        }
        cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cascade.setVelocity(1000);
    }

    public void dropBox() {
        boxpivot.setPosition(0.5);
        boxhinge.setPosition(0.2);
    }

    //Get rid of this for goToLvl
    public void goDown() {
        cascade.setTargetPosition(-4000); //Change to adapt to duckPos
        cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cascade.setVelocity(1000);
    }
}