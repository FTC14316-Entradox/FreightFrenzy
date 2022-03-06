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
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;
import java.util.List;

@Autonomous(group="Autonomous_v2.1", name="AutoB_wh_v2_1")
public class AutoB_wh_v2_1 extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/TeamShippingElement.tflite";
    private static final String[] LABELS = {
            "Team_marker"
    };
    private static final String VUFORIA_KEY = "ATTLA1L/////AAABmYwKnqPNcUUJncuM++cQUz8lRJQ7MWixmSB01lYfdLyenw+dGAK6NiZsS/bHCZHkY6+AVXp+yC8E2/AYp//YBRXhd+kFwGEHjOwVZgAf/tMT7+mORWA0akiQnpBhsNoVH1X+W5KkUKmZSzrbp3p0t5c8u5yOZNVh4BeF0hQ12LxXOdpYjIC2oj9s4OekK9eS8BfWiXpiFZnoCff0cRZFihdo9leX/rBs3HQtntdKI1vqAgeLaHef+q8PyiZm/Y5/urT9l9mBokI+I6SKa0su62HCnlF545Zr9nuugON0BevUt4GNIhH96FWp+gHjsd2rCkiCk5AigFXKsPl0md66DOcNmv1LxStR0tHVwckGCd5f";
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    public List updatedRecognitions;

    public int duckPos = 0;

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);

        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-67, 12, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-48.5, -10, Math.toRadians(0)))
                .build();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .lineToLinearHeading(new Pose2d(-68.5, 12, Math.toRadians(90)))
                .waitSeconds(1)
                .forward(
                        22.5,
                        SampleMecanumDrive.getVelocityConstraint(15, 0.9, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        waitForStart();
        if (!isStopRequested()){
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
            telemetry.addData("Duck Position", duckPos);
            telemetry.update();
            robot.safeBox();
            drive.followTrajectorySequence(trajSeq1);
            sleep(1000);
            robot.goToLvl(duckPos);
            sleep(1700);
            robot.dropBox();
            sleep(1500);
            robot.safeBox();
            drive.followTrajectorySequence(trajSeq2);
            sleep(2000);

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
