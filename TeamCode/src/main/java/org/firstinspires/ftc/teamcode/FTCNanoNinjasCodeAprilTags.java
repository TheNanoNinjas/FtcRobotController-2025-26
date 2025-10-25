// 2025-26 code for 25756 Nano Ninjas

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "ArnavMahithAprilTag")
public class FTCNanoNinjasCodeAprilTags extends LinearOpMode {

    // ===== Motor Declarations =====
    private DcMotor intakeMotor;
    private DcMotor leftShooter;
    private DcMotor rightShooter;
    private DcMotor fl_motor;
    private DcMotor fr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;
    private DcMotor rampMotor;

    // ===== AprilTag and Vision Variables =====
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean cameraStreaming = false;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // AprilTag IDs
    private int APRIL_TAG_ID_PGP = 22;
    private int APRIL_TAG_ID_PPG = 23;
    private int APRIL_TAG_ID_GPP = 21;
    private int Artifact_Bucket_ID = 24;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize all motors and camera
        initializeHardware();
        initAprilTag();

        waitForStart();

        while (opModeIsActive()) {
            // Uncomment any functions you want active during TeleOp:
            // moveRobot();
            // shootArtifacts();
            // shootArtifactsTrigger();
            // intakeArtifacts();
            // moveArtifacts();
            telemetryAprilTag();
        }
    }

    // ===== Robot Driving Control =====
    private void moveRobot() {
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        double powerScale = 0.40;

        double leftFrontPower = (drive + strafe + turn) * powerScale;
        double rightFrontPower = (drive - strafe - turn) * powerScale;
        double leftRearPower = (drive - strafe + turn) * powerScale;
        double rightRearPower = (drive + strafe - turn) * powerScale;

        // Normalize power so no wheel exceeds 1.0
        double maxPower = Math.max(Math.abs(leftFrontPower),
                Math.max(Math.abs(rightFrontPower),
                        Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower))));
        if (maxPower > 1) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftRearPower /= maxPower;
            rightRearPower /= maxPower;
        }

        // Set motor power
        fl_motor.setPower(leftFrontPower);
        fr_motor.setPower(rightFrontPower);
        bl_motor.setPower(-leftRearPower);
        br_motor.setPower(-rightRearPower);
    }

    // ===== Initialize All Hardware =====
    private void initializeHardware() {
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        rampMotor = hardwareMap.get(DcMotor.class, "rampMotor");
        fl_motor = hardwareMap.get(DcMotor.class, "fl_motor");
        fr_motor = hardwareMap.get(DcMotor.class, "fr_motor");
        bl_motor = hardwareMap.get(DcMotor.class, "bl_motor");
        br_motor = hardwareMap.get(DcMotor.class, "br_motor");

        // Safety direction settings
        fr_motor.setDirection(DcMotor.Direction.REVERSE);
        br_motor.setDirection(DcMotor.Direction.REVERSE);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Confirm motors are connected
        telemetry.addData("Hardware", "Initialized");
        telemetry.update();
    }

    // ===== Artifact Shooter Control =====
    private void shootArtifacts() {
        if (gamepad2.right_bumper) {
            leftShooter.setPower(1.0);
            rightShooter.setPower(1.0);
        } else {
            leftShooter.setPower(0.0);
            rightShooter.setPower(0.0);
        }
    }

    // Trigger-controlled shooting
    private void shootArtifactsTrigger() {
        float factor = 1;
        float shooting = gamepad2.right_trigger * factor;
        leftShooter.setPower(shooting);
        rightShooter.setPower(shooting);
    }

    // Intake control
    private void intakeArtifacts() {
        if (gamepad2.left_bumper) {
            intakeMotor.setPower(0.75);
        } else {
            intakeMotor.setPower(0.0);
        }
    }

    // Ramp control
    private void moveArtifacts() {
        if (gamepad2.triangle) {
            rampMotor.setPower(1.0);
        } else {
            rampMotor.setPower(0.0);
        }
    }

    // ===== Initialize AprilTag System =====
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        // SAFELY create VisionPortal (works on Control Hub and phone)
        int viewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VisionPortal.Builder builder;
        if (viewId != 0) {
            // Newer SDKs rename this method to setLiveViewContainerId(...)
            builder = new VisionPortal.Builder().setLiveViewContainerId(viewId);
        } else {
            builder = new VisionPortal.Builder(); // no preview (Control Hub)
        }

        // Set the camera source
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Add AprilTag processing
        builder.addProcessor(aprilTag);

        // Build the Vision Portal
        visionPortal = builder.build();
    }

    // ===== AprilTag Telemetry =====
    private void telemetryAprilTag() {
        // Only resume streaming if we aren't already streaming
        if (gamepad2.dpad_down) {
            if (!cameraStreaming) {
                visionPortal.resumeStreaming();
                cameraStreaming = true;
                sleep(50); // give camera a moment to start
            }

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                            detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)",
                            detection.center.x, detection.center.y));
                }
            }

            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");
            telemetry.update();

        } else {
            // Only stop streaming if we were streaming
            if (cameraStreaming) {
                visionPortal.stopStreaming();
                cameraStreaming = false;
            }
        }
    }
}
