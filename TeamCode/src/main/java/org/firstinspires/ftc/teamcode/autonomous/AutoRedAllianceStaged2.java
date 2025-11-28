package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.ArtifactPusher;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;

@Autonomous(name = "Auto Red Alliance Staged 2", group = "Competition")
public class AutoRedAllianceStaged2 extends OpMode {

    private enum STAGE {
        IDLE,
        MOVE_TO_TARGET,
        TURN_TO_SHOOTING_ANGLE,
        LAUNCH_ARTIFACTS,
        TURN_TO_ZERO_STAGE,
        MOVE_TO_SECOND_TARGET,
        TURN_TO_270_STAGE,
        INTAKE_ARTIFACTS,
        MOVE_INTAKE_STAGE,
        STOP
    }

    private RobotHardware robot = new RobotHardware();
    private MecanumDrive drive;
    private Shooter shooter;
    private ArtifactPusher artifactPusher;
    private Intaker intake;
    private GoBildaPinpointDriver odo;
    private Rev2mDistanceSensor distanceSensor;

    private static final double KP = 0.10;
    private static final double OBSTACLE_DISTANCE = 6.0;
    private static final double TURN_TOLERANCE_DEG = 2.0;

    private STAGE currentStage = STAGE.IDLE;
    private int nextStageIndex = 1; // for compatibility with integer stage values if desired
    private ElapsedTime stageTimer = new ElapsedTime();

    // Stage parameters (example values copied and cleaned up from your original)
    private static final double FIRST_MOVE_Y = 10.0;     // inches
    private static final double SECOND_MOVE_Y = 24.0;    // inches
    private static final double INTAKE_MOVE_Y = 43.0;    // inches

    @Override
    public void init() {
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot);
        shooter = new Shooter(robot);
        artifactPusher = new ArtifactPusher(robot);
        intake = new Intaker(robot);

        initializeSensors();

        // Reset odometry and IMU once at init so headings start at 0 when robot faces forward
        resetOdometryAtStart();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        currentStage = STAGE.IDLE;
    }

    @Override
    public void start() {
        stageTimer.reset();
        // Start the run sequence: stage numbers chosen to mirror your original flow
        currentStage = STAGE.MOVE_TO_TARGET;
        nextStageIndex = 2;

        // Make sure odometry has a stable reading at the start of auto
        resetOdometryAtStart();
    }

    @Override
    public void loop() {
        try {
            switch (currentStage) {
                case MOVE_TO_TARGET:
                    moveToTargetStage(FIRST_MOVE_Y, STAGE.TURN_TO_SHOOTING_ANGLE);
                    break;

                case TURN_TO_SHOOTING_ANGLE:
                    turnToHeadingStage(-45, STAGE.LAUNCH_ARTIFACTS);
                    break;

                case LAUNCH_ARTIFACTS:
                    launchArtifactsStage(STAGE.TURN_TO_ZERO_STAGE);
                    break;

                case TURN_TO_ZERO_STAGE:
                    turnToHeadingStage(18, STAGE.MOVE_TO_SECOND_TARGET);
                    break;

                case MOVE_TO_SECOND_TARGET:
                    moveToTargetStage(SECOND_MOVE_Y, STAGE.TURN_TO_270_STAGE);
                    break;

                case TURN_TO_270_STAGE:
                    turnToHeadingStage(93, STAGE.INTAKE_ARTIFACTS);
                    break;

                case INTAKE_ARTIFACTS:
                    startIntake(STAGE.MOVE_INTAKE_STAGE);
                    break;

                case MOVE_INTAKE_STAGE:
                    moveToTargetStage(-24, STAGE.STOP);
                    break;

                case STOP:
                default:
                    drive.stop();
                    shooter.stopShooting();
                    intake.stopPushing();
                    artifactPusher.stopPushing();
                    break;
            }
        } catch (Exception e) {
            telemetry.addData("Exception", e.getMessage());
        }

        telemetry.addData("Stage", currentStage.name());
        telemetry.update();
    }

    // ----------------- Helpers and Stage Implementations -----------------

    private void initializeSensors() {
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        // Set offsets and directions according to your hardware. Keep heading "forward = 0".
        odo.setOffsets(-88, 0.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    /**
     * Use this only at the *start* of autonomous to zero position/heading.
     * Do NOT call this after turns — resetting the IMU will zero the heading and break relative turns.
     */
    private void resetOdometryAtStart() {
        odo.resetPosAndIMU();
        try { Thread.sleep(50); } catch (InterruptedException ignored) {}
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        try { Thread.sleep(50); } catch (InterruptedException ignored) {}
        odo.update();
    }

    private double angleError(double targetDeg, double currentDeg) {
        double error = targetDeg - currentDeg;
        // Normalize to [-180,180)
        error = ((error + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        return error;
    }

    private void turnToHeadingStage(double targetHeading, STAGE nextStage) {
        // Update odometry to read fresh heading
        odo.update();
        double currentHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
        double error = angleError(targetHeading, currentHeading);

        telemetry.addData("Target Heading", targetHeading);
        telemetry.addData("Current Heading", currentHeading);
        telemetry.addData("Turn Error", error);

        if (Math.abs(error) <= TURN_TOLERANCE_DEG) {
            drive.stop();
            currentStage = nextStage;
            stageTimer.reset();
            // Do NOT reset odometry here — keeps heading continuity for next actions
            return;
        }

        double turnPower;
        double absError = Math.abs(error);
        if (absError > 30) turnPower = 0.40;
        else if (absError > 10) turnPower = 0.25;
        else turnPower = 0.15;

        turnPower *= Math.signum(error);

        robot.fl_motor.setPower(-turnPower);
        robot.bl_motor.setPower(-turnPower);
        robot.fr_motor.setPower(turnPower);
        robot.br_motor.setPower(turnPower);
    }

    private void moveToTargetStage(double targetY, STAGE nextStage) {
        odo.update();
        Pose2D pos = odo.getPosition();

        double currentY = pos.getY(DistanceUnit.INCH);
        // Allowing negative/positive movements — targetY can be negative for reverse
        double error = targetY - currentY;

        double drivePower = error * KP;
        drivePower = Math.max(-0.6, Math.min(0.6, drivePower));

        double distanceInches;
        try {
            distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);
        } catch (Exception e) {
            distanceInches = Double.POSITIVE_INFINITY; // sensor failure: ignore obstacle
        }

        boolean targetReached = Math.abs(error) < 1.0;
        boolean obstacleClose = distanceInches < OBSTACLE_DISTANCE;

        telemetry.addData("Current Y (in)", "%.2f", currentY);
        telemetry.addData("Error Y (in)", "%.2f", error);
        telemetry.addData("Distance Sensor (in)", "%.2f", distanceInches);

        if (targetReached || obstacleClose) {
            drive.stop();
            if (obstacleClose) {
                // back away a small amount safely (blocking short wait)
                driveBackward(0.25, 300);
            }
            currentStage = nextStage;
            stageTimer.reset();
            // Reset position origin for the next straight movement if desired but don't reset heading
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, odo.getPosition().getHeading(AngleUnit.DEGREES)));
            odo.update();
            return;
        }

        // Drive forward/backward based on sign of drivePower
        if (drivePower >= 0) {
            driveForward(drivePower);
        } else {
            driveBackward(Math.abs(drivePower));
        }
    }

    private void launchArtifactsStage(STAGE nextStage) {
        double t = stageTimer.seconds();
        if (t < 1.0) {
            shooter.startShootingFar();
        } else if (t < 2.0) {
            artifactPusher.startWheel();
        } else if (t < 4.0) {
            intake.startPushing();
            artifactPusher.startWheel();
        } else {
            shooter.stopShooting();
            intake.stopPushing();
            artifactPusher.stopPushing();
            currentStage = nextStage;
            stageTimer.reset();
            // keep odometry heading, but we can zero Y if desired for next move:
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, odo.getPosition().getHeading(AngleUnit.DEGREES)));
            odo.update();
        }

        telemetry.addData("Launch Timer", "%.1f", t);
    }

    private void startIntake(STAGE nextStage) {
        intake.startPushing();
        currentStage = nextStage;
        stageTimer.reset();
        // Keep heading consistent
    }

    private void stopIntake(STAGE nextStage) {
        intake.stopPushing();
        currentStage = nextStage;
        stageTimer.reset();
    }

    // Basic drive helpers
    private void driveForward(double power) {
        robot.fl_motor.setPower(power);
        robot.fr_motor.setPower(power);
        robot.bl_motor.setPower(power);
        robot.br_motor.setPower(power);
    }

    private void driveBackward(double power, long timeMs) {
        driveBackward(power);
        try { Thread.sleep(timeMs); } catch (InterruptedException ignored) {}
        drive.stop();
    }

    private void driveBackward(double power) {
        robot.fl_motor.setPower(-power);
        robot.fr_motor.setPower(-power);
        robot.bl_motor.setPower(-power);
        robot.br_motor.setPower(-power);
    }
}
