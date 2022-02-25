package org.firstinspires.ftc.teamcode.TeleOp.v2_0;

import org.firstinspires.ftc.teamcode.threadopmode.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group="TeleOp_v2", name="TeleOp_v2_1")
public class TeleOp_v2_1 extends ThreadOpMode {

    //Declaring drivetrain members
    public DcMotor fr = null;
    public DcMotor fl = null;
    public DcMotor bl = null;
    public DcMotor br = null;
    public DcMotor mr = null;
    public DcMotor ml = null;

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

                double yAdj = y*Math.abs(y)*Math.abs(y);
                double xAdj = x*Math.abs(x)*Math.abs(x);

                if (y == 0.0) {
                    fl.setPower(xAdj);
                    bl.setPower(-xAdj);
                    fr.setPower(-xAdj);
                    br.setPower(xAdj);
                }

                if (x == 0.0) {
                    fl.setPower(yAdj);
                    bl.setPower(yAdj);
                    fr.setPower(yAdj);
                    br.setPower(yAdj);
                }

                if (Math.abs(rx) > 0) {
                    fl.setPower(rx);
                    bl.setPower(rx);
                    fr.setPower(-rx);
                    br.setPower(-rx);
                }
            }
        }));

        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {

                while (gamepad1.b ) { car.setPower(-0.6); }
                while (gamepad1.x) { car.setPower(0.6); }
                car.setPower(0);

                while (gamepad1.right_bumper) { intake.setPower(0.9); }
                while (gamepad1.left_bumper) { intake.setPower(-0.9); }

                telemetry.update();

            }
        }));

        //Driver 2
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {

                if (gamepad2.right_bumper) {
                    boxpivot.setPosition(0.25);
                    boxhinge.setPosition(0.0);
                }

                if (gamepad2.left_bumper) {
                    boxpivot.setPosition(0.5);
                    boxhinge.setPosition(0.175);
                }

                if (gamepad2.a) {
                    boxpivot.setPosition(0.2);
                    boxhinge.setPosition(0.2);
                }

                if (gamepad2.b) {
                    boxpivot.setPosition(0.2);
                    boxhinge.setPosition(0.7);
                }

                //intake
                while (gamepad2.dpad_down) { pivot.setPower(0.8); }
                while (gamepad2.dpad_up) { pivot.setPower(-0.8); }

                pivot.setPower(0);
                intake.setPower(0);
            }

        }));

        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {

                double ry = gamepad2.right_stick_y;
                if (Math.abs(ry) > 0) { cascade.setPower(-ry); }
                cascade.setPower(0.0);
                telemetry.addData("box pivot servo position", boxpivot.getPosition());
                telemetry.addData("box hinge servo position", boxhinge.getPosition());
                telemetry.update();
            }

        }));

    }

    @Override
    public void mainLoop() {
        //No need
    }
}
