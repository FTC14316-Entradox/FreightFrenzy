package org.firstinspires.ftc.teamcode.Autonomous.v2_0;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(group="Autonomous_v2.2", name="AutoB_car_v2_2")
public class AutoB_car_v2_2 extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY = "ATTLA1L/////AAABmYwKnqPNcUUJncuM++cQUz8lRJQ7MWixmSB01lYfdLyenw+dGAK6NiZsS/bHCZHkY6+AVXp+yC8E2/AYp//YBRXhd+kFwGEHjOwVZgAf/tMT7+mORWA0akiQnpBhsNoVH1X+W5KkUKmZSzrbp3p0t5c8u5yOZNVh4BeF0hQ12LxXOdpYjIC2oj9s4OekK9eS8BfWiXpiFZnoCff0cRZFihdo9leX/rBs3HQtntdKI1vqAgeLaHef+q8PyiZm/Y5/urT9l9mBokI+I6SKa0su62HCnlF545Zr9nuugON0BevUt4GNIhH96FWp+gHjsd2rCkiCk5AigFXKsPl0md66DOcNmv1LxStR0tHVwckGCd5f";
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    public List updatedRecognitions;
    public int duckPos;
    public int num = 1;
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        Robot_v2 robot2 = new Robot_v2(hardwareMap);

        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        waitForStart();
        if (!isStopRequested()) {
            robot.parallelBox();
            sleep(2500);
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                telemetry.addData("Duck Pos", updatedRecognitions);
                telemetry.update();
                if (updatedRecognitions != null) {
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions){
                        if (recognition.getLabel() == "Team_marker") {
                            if (recognition.getLeft() < 374) {
                                duckPos = 2;
                                if (recognition.getLeft() < 177) { duckPos = 1; }
                            }
                        }
                    }
                    i++;
                }
            }
            if (duckPos == 0) {
                duckPos = 3;
            }
            telemetry.addData("Marker Position", duckPos);
            telemetry.update();

            robot.safeBox();
            runtime.reset();

            while (runtime.milliseconds() < 300) {
                robot.drive(0.5, 1);
            }
            robot.stop();
            while (runtime.milliseconds() < 3000 && runtime.milliseconds() > 1500) {
                robot.drive(0.5, 2);
            }
            robot.stop();
            while (runtime.milliseconds() < 6000 && runtime.milliseconds() > 5000) {
                robot.drive(-0.3, 1);
            }
            robot.stop();
            while (runtime.milliseconds() < 12000 && runtime.milliseconds() > 8000) {
                robot.carousel(0.6);
            }
            while (runtime.milliseconds() < 16000 && runtime.milliseconds() > 14000) {
                robot.drive(0.5, 1);
            }
            robot.stop();

            drive.turn(Math.toRadians(90));
            while (runtime.milliseconds() < 22000 && runtime.milliseconds() > 20000) {
                robot.drive(0.5, 1);
            }
            robot.stop();

            robot2.setCascade(duckPos);
            robot2.dropBox();
            robot2.pause(1500);
            robot2.safeBox();

            while (runtime.milliseconds() < 28000 && runtime.milliseconds() > 26000) {
                robot.drive(-0.5, 1);
            }
            robot.stop();

            while (runtime.milliseconds() < 29000 && runtime.milliseconds() > 28500) {
                robot.drive(-0.5, 2);
            }
            robot.stop();


        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcamback");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //To turn off object tracker: tfodParameters.useObjectTracker = false;
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile("TeamShippingElement.tflite", LABELS); //Change to loadModelFromAsset() if using
    }
}