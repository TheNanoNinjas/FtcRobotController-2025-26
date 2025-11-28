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
        TURN_TO_90_STAGE,
        INTAKE_ARTIFACTS,
        MOVE_INTAKE_STAGE,
        STOP_INTAKE,
        TURN_TO_ZERO,
        MOVE_BACK_TO_SHOOT,
        TURN_TO_SHOOTING_ANGLE2,
        SHOOT_ARTIFACTS,
        MOVE_AFTER_INTAKE,
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
    private ElapsedTime stageTimer = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap);

        drive = new MecanumDrive(robot);
        shooter = new Shooter(robot);
        artifactPusher = new ArtifactPusher(robot);
        intake = new Intaker(robot);

        initializeSensors();
        resetOdometryAtStart();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        currentStage = STAGE.IDLE;
    }

    @Override
    public void start() {
        stageTimer.reset();
        currentStage = STAGE.MOVE_TO_TARGET;

        resetOdometryAtStart();
    }

    @Override
    public void loop() {
        try {
            switch (currentStage) {
                case MOVE_TO_TARGET:
                    moveToTargetStage(10, STAGE.TURN_TO_SHOOTING_ANGLE);
                    break;

                case TURN_TO_SHOOTING_ANGLE:
                    turnToHeadingStage(338, STAGE.LAUNCH_ARTIFACTS);
                    break;

                case LAUNCH_ARTIFACTS:
                    launchArtifactsStage(STAGE.TURN_TO_ZERO_STAGE);
                    break;

                case TURN_TO_ZERO_STAGE:
                    turnToHeadingStage(0, STAGE.MOVE_TO_SECOND_TARGET);
                    break;

                case MOVE_TO_SECOND_TARGET:
                    moveToTargetStage(24, STAGE.TURN_TO_90_STAGE);
                    break;

                case TURN_TO_90_STAGE:
                    turnToHeadingStage(90, STAGE.INTAKE_ARTIFACTS);
                    break;

                case INTAKE_ARTIFACTS:
                    startIntake(STAGE.MOVE_INTAKE_STAGE);
                    break;

                case MOVE_INTAKE_STAGE:
                    moveBackwardsToTargetStage(-24, STAGE.MOVE_AFTER_INTAKE);
                    break;

                case MOVE_AFTER_INTAKE:
                    moveToTargetStage(24, STAGE.STOP_INTAKE);
                    break;

                case STOP_INTAKE:
                    stopIntake(STAGE.TURN_TO_ZERO);
                    break;

                case TURN_TO_ZERO:
                    turnToHeadingStage(0, STAGE.MOVE_BACK_TO_SHOOT);
                    break;

                case MOVE_BACK_TO_SHOOT:
                    moveToTargetStage(-24, STAGE.TURN_TO_SHOOTING_ANGLE2);
                    break;

                case TURN_TO_SHOOTING_ANGLE2:
                    turnToHeadingStage(338, STAGE.SHOOT_ARTIFACTS);
                    break;

                case SHOOT_ARTIFACTS:
                    launchArtifactsStage(STAGE.STOP);
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


    private void initializeSensors() {
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(-88, 0.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    private void resetOdometryAtStart() {
        odo.resetPosAndIMU();
        sleep(50);
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        sleep(50);
        odo.update();
    }

    private void sleep(long ms) {
        try { Thread.sleep(ms); } catch (Exception ignored) {}
    }

    private double angleError(double targetDeg, double currentDeg) {
        double error = targetDeg - currentDeg;
        return ((error + 180) % 360 + 360) % 360 - 180;
    }

    private void turnToHeadingStage(double targetHeading, STAGE nextStage) {
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

        double error = targetY - currentY;

        double drivePower = Math.max(-0.35, Math.min(0.35, error * KP));

        double distanceInches;
        try {
            distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);
        } catch (Exception e) {
            distanceInches = Double.POSITIVE_INFINITY;
        }

        boolean targetReached = Math.abs(error) < 2.0;
        boolean obstacleClose = distanceInches < OBSTACLE_DISTANCE;

        if (targetReached || obstacleClose) {
            drive.stop();

            if (obstacleClose) driveBackward(0.25, 300);

            currentStage = nextStage;
            stageTimer.reset();

            // Reset Y but keep heading
            odo.setPosition(new Pose2D(
                    DistanceUnit.INCH, 0, 0,
                    AngleUnit.DEGREES,
                    odo.getPosition().getHeading(AngleUnit.DEGREES)
            ));
            odo.update();
            return;
        }

        if (drivePower >= 0) driveForward(drivePower);
        else driveBackward(-drivePower);

        telemetry.addData("Current Y", currentY);
        telemetry.addData("Error", error);
        telemetry.addData("Drive Power", drivePower);
        telemetry.addData("Distance Sensor", distanceInches);
    }

    private void moveBackwardsToTargetStage(double targetY, STAGE nextStage) {
        odo.update();
        Pose2D pos = odo.getPosition();

        double currentY = pos.getY(DistanceUnit.INCH);

        // Correct backward error calculation
        double error = targetY - currentY;   // targetY MUST be negative for backwards

        // Proportional backwards power
        double drivePower = error * KP;
        drivePower = Math.max(-0.3, Math.min(0.3, drivePower));

        double distanceInches;
        try {
            distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);
        } catch (Exception e) {
            distanceInches = Double.POSITIVE_INFINITY;
        }

        boolean targetReached = Math.abs(error) < 1.0;
        boolean obstacleClose = distanceInches < OBSTACLE_DISTANCE;

        if (targetReached || obstacleClose) {
            drive.stop();

            if (obstacleClose) {
                driveBackward(0.25, 300);
            }

            currentStage = nextStage;
            stageTimer.reset();

            // reset Y origin only
            odo.setPosition(new Pose2D(
                    DistanceUnit.INCH,
                    0, 0,
                    AngleUnit.DEGREES,
                    pos.getHeading(AngleUnit.DEGREES)
            ));
            odo.update();

            return;
        }

        // --- Drive backwards ---
        driveBackward(Math.abs(drivePower));
        intake.startPushing();

        telemetry.addData("CurrentY", currentY);
        telemetry.addData("TargetY", targetY);
        telemetry.addData("Error", error);
        telemetry.addData("DrivePower", drivePower);
        telemetry.addData("DistanceSensor", distanceInches);
    }


    private void launchArtifactsStage(STAGE nextStage) {
        double t = stageTimer.seconds();

        if (t < 1.0) shooter.startShootingFar();
        else if (t < 2.0) artifactPusher.startWheel();
        else if (t < 4.0) {
            intake.startPushing();
            artifactPusher.startWheel();
        } else {
            shooter.stopShooting();
            intake.stopPushing();
            artifactPusher.stopPushing();

            currentStage = nextStage;
            stageTimer.reset();

            odo.setPosition(new Pose2D(
                    DistanceUnit.INCH, 0, 0,
                    AngleUnit.DEGREES,
                    odo.getPosition().getHeading(AngleUnit.DEGREES)
            ));
            odo.update();
        }

        telemetry.addData("Launch Timer", t);
    }

    private void startIntake(STAGE nextStage) {
        intake.startPushing();
        currentStage = nextStage;
        stageTimer.reset();
    }

    private void stopIntake(STAGE nextStage) {
        intake.stopPushing();
        currentStage = nextStage;
        stageTimer.reset();
    }


    private void driveForward(double power) {
        robot.fl_motor.setPower(power);
        robot.fr_motor.setPower(power);
        robot.bl_motor.setPower(power);
        robot.br_motor.setPower(power);
    }

    private void driveBackward(double power, long timeMs) {
        driveBackward(power);
        sleep(timeMs);
        drive.stop();
    }

    private void driveBackward(double power) {
        robot.fl_motor.setPower(-power);
        robot.fr_motor.setPower(-power);
        robot.bl_motor.setPower(-power);
        robot.br_motor.setPower(-power);
    }
}
