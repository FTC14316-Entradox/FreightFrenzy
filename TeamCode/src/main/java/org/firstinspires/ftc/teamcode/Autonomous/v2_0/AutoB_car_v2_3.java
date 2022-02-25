package org.firstinspires.ftc.teamcode.Autonomous.v2_0;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.util.List;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Autonomous(group="Autonomous_v2.0", name="AutoB_car_v2_3")
public class AutoB_car_v2_3 extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite"; //Change ASAP
    private static final String[] LABELS = { //Change ASAP
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    //Make sure token will not expire
    private static final String VUFORIA_KEY = "ATTLA1L/////AAABmYwKnqPNcUUJncuM++cQUz8lRJQ7MWixmSB01lYfdLyenw+dGAK6NiZsS/bHCZHkY6+AVXp+yC8E2/AYp//YBRXhd+kFwGEHjOwVZgAf/tMT7+mORWA0akiQnpBhsNoVH1X+W5KkUKmZSzrbp3p0t5c8u5yOZNVh4BeF0hQ12LxXOdpYjIC2oj9s4OekK9eS8BfWiXpiFZnoCff0cRZFihdo9leX/rBs3HQtntdKI1vqAgeLaHef+q8PyiZm/Y5/urT9l9mBokI+I6SKa0su62HCnlF545Zr9nuugON0BevUt4GNIhH96FWp+gHjsd2rCkiCk5AigFXKsPl0md66DOcNmv1LxStR0tHVwckGCd5f";
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    public List updatedRecognitions;
    public int duckPos;

    @Override
    public void runOpMode() {

        Robot_v2 robot = new Robot_v2(hardwareMap);

        initVuforia();
        initTfod();
        if (tfod != null) { tfod.activate(); }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        waitForStart();
        robot.safeBox();
        if (!isStopRequested()){
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                //telemetry.addData("Duck Pos", updatedRecognitions);
                //telemetry.update();
                if (updatedRecognitions != null) {
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions){
                        if (recognition.getLabel() == "Cube") {
                            if (recognition.getLeft() < 220) { duckPos = 1; }
                            else {
                                if (recognition.getLeft() < 900) { duckPos = 2; }
                            }
                        }
                    }
                    i++;
                }
            }
            if (duckPos == 0) {
                duckPos = 3;
            }
            telemetry.addData("Duck Position", duckPos);
            telemetry.update();

            robot.forward(0.6, 10);
            robot.strafe(0.6, 1);
            robot.forward(-0.1, 0.1);
            robot.carouselTime(0.6, 2000);
            robot.forward(0.6, 1);
            drive.turn(Math.toRadians(90));
            robot.forward(0.6, 0.5);
            robot.setCascade(duckPos);
            robot.pause(3000);
            robot.dropBox();
            robot.setCascade(0);
            robot.forward(-0.6, 1);
            robot.strafe(-0.3, 0.5);
            robot.brake();
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
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}