package org.firstinspires.ftc.teamcode.PartsTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.lang.reflect.Array;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(group="PartsTests", name="ColorSensorTest")
public class ColorSensorTest extends OpMode{

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

    //runtime variable
    public ElapsedTime runtime = new ElapsedTime();

    //Declaring carousel members
    public CRServo car;

    //Declaring cascading slide members
    public CRServo cascade1;
    public CRServo cascade2;
    public Servo dump = null;

    //Declaring color sensor
    public NormalizedColorSensor colorSensor = null;
    public float[] hsvValues = null;

    //Hardware initialization code
    public void init() {

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

        //color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorback");
    }

    public void start(){
        runtime.reset();
    }

    @Override
    public void loop() {

        while(((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)>10.00){
            fl.setPower(0.8);
            fr.setPower(0.8);
            bl.setPower(0.8);
            br.setPower(0.8);
        }

        fl.setPower(0.0);
        fr.setPower(0.0);
        bl.setPower(0.0);
        br.setPower(0.0);

        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float gain = 2;

        final float[] hsvValues = new float[3];

        boolean xButtonPreviouslyPressed = false;
        boolean xButtonCurrentlyPressed = false;

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);


        if (colorSensor instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        }

        telemetry.addData("Status","runtime"+ runtime.toString());
        telemetry.update();
    }

    @Override
    public void stop(){

    }
}