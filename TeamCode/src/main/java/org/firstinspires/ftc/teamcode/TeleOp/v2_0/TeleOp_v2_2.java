package org.firstinspires.ftc.teamcode.TeleOp.v2_0;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Autonomous.v2_0.Robot_v2;

@Disabled
@TeleOp(group="TeleOp_v2", name="TeleOp_v2_2")
public class TeleOp_v2_2 extends LinearOpMode {

    public DcMotor fr;
    public DcMotor fl;
    public DcMotor bl;
    public DcMotor br;

    public DcMotor car;
    public DcMotor pivot;
    public DcMotor intake;
    public Servo boxhinge;
    public Servo boxpivot;

    Robot_v2 robot = new Robot_v2(hardwareMap);

    @Override
    public void runOpMode() {

        fr  = hardwareMap.get(DcMotor.class, "frontright");
        fl  = hardwareMap.get(DcMotor.class, "frontleft");
        bl  = hardwareMap.get(DcMotor.class, "backleft");
        br  = hardwareMap.get(DcMotor.class, "backright");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        car = hardwareMap.get(DcMotor.class, "carousel");
        pivot  = hardwareMap.get(DcMotor.class, "pivot");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        boxhinge = hardwareMap.get(Servo.class, "box");
        boxpivot = hardwareMap.get(Servo.class, "boxpivot");

        Thread driveThread = new DriveThread();
        Thread intakeThread = new intakeThread();
        Thread boxThread = new boxThread();
        Thread cascadeThread = new cascadeThread();

        telemetry.addData("Mode: ", "Waiting");
        telemetry.update();

        waitForStart();
        telemetry.addData("Mode: ", "Started");
        telemetry.update();

        robot.safeBox();

        driveThread.start();
        intakeThread.start();
        boxThread.start();
        cascadeThread.start();

        while (opModeIsActive()) {
            telemetry.addData("Mode: ", "running");
            telemetry.update();

            idle();
        }

        driveThread.interrupt();
        intakeThread.interrupt();
        boxThread.interrupt();
        cascadeThread.interrupt();

    }

    private class DriveThread extends Thread {
        public DriveThread() { this.setName("DriveThread"); }

        @Override
        public void run() {
            while (!isInterrupted()) {

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                double yAdj = y*Math.abs(y);
                double xAdj = y*Math.abs(y);

                if (y == 0.0) {
                    robot.strafe(xAdj, 0.1);
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

                idle();
            }
        }
    }

    private class intakeThread extends Thread {
        public intakeThread() { this.setName("intakeThread"); }

        @Override
        public void run() {
            while (!isInterrupted()) {

                while (gamepad1.b ) { car.setPower(-0.6); }
                while (gamepad1.x) { car.setPower(0.6); }
                car.setPower(0);

                while (gamepad1.right_bumper) { intake.setPower(0.9); }
                while (gamepad1.left_bumper) { intake.setPower(-0.9); }
                intake.setPower(0);

                idle();
            }
        }
    }

    private class boxThread extends Thread {
        public boxThread() { this.setName("boxThread"); }

        @Override
        public void run() {
            while (!isInterrupted()) {

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

                while (gamepad2.dpad_down) { pivot.setPower(0.8); }
                while (gamepad2.dpad_up) { pivot.setPower(-0.8); }

                pivot.setPower(0);

                idle();
            }
        }
    }

    private class cascadeThread extends Thread {
        public cascadeThread() { this.setName("cascadeThread"); }

        @Override
        public void run() {
            while (!isInterrupted()) {

                if (gamepad2.y) { robot.setCascade(3); }
                if (gamepad2.x) { robot.setCascade(2); }
                //if (gamepad2.y) { robot.setCascade(1); }
                //if (gamepad2.y) { robot.setCascade(0); }

                idle();
            }
        }
    }
}
