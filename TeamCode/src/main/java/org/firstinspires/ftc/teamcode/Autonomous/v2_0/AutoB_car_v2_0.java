package org.firstinspires.ftc.teamcode.Autonomous.v2_0;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.util.List;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(group="Autonomous_v2.0", name="AutoB_car_v2_0")
public class AutoB_car_v2_0 extends LinearOpMode {

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

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);

        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-68, -36, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-56, -70, Math.toRadians(0)))
                .back(1)
                .addDisplacementMarker(() -> {
                    robot.carousel(0.6);
                    sleep(2000);
                    robot.carousel(0);
                })
                .lineToSplineHeading(new Pose2d(-44, -18, Math.toRadians(0))) //-51
                .addDisplacementMarker(() -> {
                    robot.goToLvl(duckPos);
                    sleep(4000);
                    robot.dropBox();
                    sleep(1500);
                    robot.safeBox();
                }) //Make sure doesn't pass until marker is done (split trajectories)
                //.lineToSplineHeading(new Pose2d(-70, 0, Math.toRadians(90))) //Go to corner
                .lineToLinearHeading(new Pose2d(-72, 0, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    //sleep(1000);
                    robot.goDown(); //Go down
                })
                .forward(30) //Forward
                .build();

        waitForStart();
        if (!isStopRequested()){
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                telemetry.addData("Duck Pos", updatedRecognitions);
                telemetry.update();
                if (updatedRecognitions != null) {
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions){
                        if (recognition.getLabel() == "Cube") {
                            if (recognition.getLeft() < 220) {
                                duckPos = 1;
                            }
                            else {
                                if (recognition.getLeft() < 900) {
                                    duckPos = 2;
                                }
                            }
                        }
                    }
                    i++;
                }
            }
            if (duckPos == 0) {
                duckPos = 3;
            }
            robot.safeBox();
            telemetry.addData("Duck Position", duckPos);
            telemetry.update();
            drive.followTrajectorySequence(trajSeq);
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