//Red alliance, starting near the storage unit, line up on the penultimate tile, spin the carousel, park in the storage unit. 

//.adb connect 192.168.49.1:5555 > driver hub
//.adb connect 192.168.43.1:5555 > control hub

package org.firstinspires.ftc.teamcode.Autonomous.v1_0;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.lang.reflect.Array;
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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group="Autonomous_v1", name="AutoR_su_16pts")
@Disabled
public class AutoR_su_16pts extends OpMode{

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
    public int num = 1;
    
    //Declaring carousel members
    public CRServo car;
    
    //Declaring cascading slide members
    public CRServo cascade1;
    public CRServo cascade2;
    public Servo dump = null;
    
    //Declaring color sensor
    public NormalizedColorSensor colorSensor1 = null;
    public NormalizedColorSensor colorSensor2 = null;
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
        //mr = hardwareMap.get(DcMotor.class, "middle1");
        //ml = hardwareMap.get(DcMotor.class, "middle2");
        
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
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorfront");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorback");
    }
    
    public void start(){
        runtime.reset();
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        
        //encoders
        int encoder = fl.getCurrentPosition();
        
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        //print encoder value
        telemetry.addLine("encoder="+Integer.toString(encoder));
        telemetry.update();
        
        
        while (num==1){
        encoder = fl.getCurrentPosition();
        if (num == 1 && encoder < 30){
            fl.setPower(0.5);
            fr.setPower(0.5);
            bl.setPower(0.5);
            br.setPower(0.5);
      
            telemetry.addLine("inside if, encoder="+Integer.toString(encoder));
            telemetry.update();
            }
        else{
                num = 2; 
                fl.setPower(0.0);
                fr.setPower(0.0);
                bl.setPower(0.0);
                br.setPower(0.0);
            }
        }  
        
        int encodertemp = encoder;
        
        while (num==2){
        encoder = fl.getCurrentPosition();
        if (encoder > encodertemp-580){
            fl.setPower(-0.5);
            fr.setPower(0.5);
            bl.setPower(0.5);
            br.setPower(-0.5);
      
            telemetry.addLine("inside if, encoder="+Integer.toString(encoder));
            telemetry.update();
            }
        else{
                num = 3; 
                fl.setPower(0.0);
                fr.setPower(0.0);
                bl.setPower(0.0);
                br.setPower(0.0);
            }
        }  
        
        while (runtime.seconds()>2.50 && runtime.seconds()<3.50){
            fl.setPower(-0.3);
            fr.setPower(0.3);
            bl.setPower(0.3);
            br.setPower(-0.3);
        }
        
        fl.setPower(0.0);
        fr.setPower(0.0);
        bl.setPower(0.0);
        br.setPower(0.0);
        
        while (runtime.seconds()>3.50 && runtime.seconds()<5.90){
            car.setDirection(DcMotorSimple.Direction.REVERSE);
            car.setPower(-0.8);
        }
        
        car.setPower(0.0);
        num = 4;
        
        while (runtime.seconds()>6.00 && runtime.seconds()<7.90){
            car.setDirection(DcMotorSimple.Direction.REVERSE);
            car.setPower(-0.8);
        }
        num = 5;
        while (runtime.seconds()>8.00 && runtime.seconds()<9.90){
            car.setDirection(DcMotorSimple.Direction.REVERSE);
            car.setPower(-0.8);
        }
        
        car.setPower(0.0);
        num = 6;
        
        while (runtime.seconds()>9.90 && runtime.seconds()<10.50){
            fl.setPower(-0.4);
            fr.setPower(-0.4);
            bl.setPower(-0.4);
            br.setPower(-0.4);
        }
        
        num=7;
        fl.setPower(0.0);
        fr.setPower(0.0);
        bl.setPower(0.0);
        br.setPower(0.0);
        
        while (num == 7 && runtime.seconds()>10.60 && runtime.seconds()<11.00){
            fl.setPower(0.5);
            fr.setPower(0.5);
            bl.setPower(0.5);
            br.setPower(0.5);
        }
        num = 8;
        fl.setPower(0.0);
        fr.setPower(0.0);
        bl.setPower(0.0);
        br.setPower(0.0);
        
        while (num == 8 && runtime.seconds()>11.60 && runtime.seconds()<13.20){
            fl.setPower(-0.4);
            fr.setPower(0.4);
            bl.setPower(0.4);
            br.setPower(-0.4);
        }
        
        num = 9;
        fl.setPower(0.0);
        fr.setPower(0.0);
        bl.setPower(0.0);
        br.setPower(0.0);
        
        telemetry.addLine("outside if, encoder="+Integer.toString(encoder));
        telemetry.update();
        
        // Get the normalized colors from the sensor
      NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
      NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
       float gain = 2;

        final float[] hsvValues = new float[3];
    
        boolean xButtonPreviouslyPressed = false;
        boolean xButtonCurrentlyPressed = false;

      // Update the hsvValues array by passing it to Color.colorToHSV()
      Color.colorToHSV(colors1.toColor(), hsvValues);

      telemetry.addLine()
              .addData("Red", "%.3f", colors1.red)
              .addData("Green", "%.3f", colors1.green)
              .addData("Blue", "%.3f", colors1.blue);
      telemetry.addLine()
              .addData("Hue", "%.3f", hsvValues[0])
              .addData("Saturation", "%.3f", hsvValues[1])
              .addData("Value", "%.3f", hsvValues[2]);
      telemetry.addData("Alpha", "%.3f", colors1.alpha);

      if (colorSensor1 instanceof DistanceSensor) {
        telemetry.addData("color 1 Distance (cm)", "%.3f", ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM));
      }
      if (colorSensor2 instanceof DistanceSensor) {
        telemetry.addData("color 2 Distance (cm)", "%.3f", ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM));
        
        telemetry.addData("Status","runtime"+ runtime.toString());
        telemetry.addData("FL", fl.getCurrentPosition());
        telemetry.addData("BL", bl.getCurrentPosition());
        telemetry.addData("FR", fr.getCurrentPosition());
        telemetry.addData("BR", br.getCurrentPosition());
        
        telemetry.addData("outerloop", runtime.toString());
        telemetry.update();
        
    }
}
}