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

@Autonomous(name = "Auto Red Alliance Avi Staged", group = "Competition")
public class AutoRedStagedAvi extends OpMode {

    enum STATE  {
        MOVE_TO_TARGET, TURN_TO_SHOOTING_ANGLE, LAUNCH_ARTIFACTS, TURN_TO_ZERO_STAGE, MOVE_TO_SECOND_TARGET, TURN_TO_270_STAGE, INTAKE_ARTIFACTS, MOVE_INTAKE_STAGE
    }

    private final RobotHardware robot = new RobotHardware();
    private MecanumDrive drive;
    private Shooter shooter;
    private ArtifactPusher artifactPusherArtifacts;
    private Intaker intake;
    private GoBildaPinpointDriver odo;
    private Rev2mDistanceSensor distanceSensor;

    private static final double TARGET_Y_INCHES = 10.0;
    private static final double SECOND_TARGET_Y = 28;
    private static final double INTAKE_MOVE_Y = 43;
    private static final double SECOND_INTAKE_MOVE_Y = 43;
    private static final double KP = 0.10;
    private static final double OBSTACLE_DISTANCE = 6.0;

    private static int STAGE = 1;
    private ElapsedTime stageTimer = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot);
        shooter = new Shooter(robot);
        artifactPusherArtifacts = new ArtifactPusher(robot);
        intake = new Intaker(robot);

        initializeSensors();
        resetOdometry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        stageTimer.reset();
        resetOdometry();
        STAGE = 1;
    }

    @Override
    public void loop() {
        switch (STAGE) {
            case 1:
                moveToTargetStage(10,2);

                break;
            case 2:
                turnToHeadingStage(338,3);

                break;
            case 3:
                launchArtifactsStage();

                break;
            case 4:
                turnToHeadingStage(0,5);

                break;
            case 5:
                moveToTargetStage(28,6);

                break;
            case 6:
                turnToHeadingStage(90,7);

                break;
            case 7:
                intakeArtifactsStage();

                break;
            case 8:
                moveToTargetStage(-24,9);

                break;
            case 9:
                moveToTargetStage(24,10);

                break;
            case 10:
                turnToHeadingStage(0,11);

                break;
            case 11:
                moveToTargetStage(-28,12);

                break;
            case 12:
                turnToHeadingStage(338,13);

                break;
            case 13:
                launchArtifactsStage();

                break;

            default:
                drive.stop();
                shooter.stopShooting();
                intake.stopPushing();
                artifactPusherArtifacts.stopPushing();
                break;
        }

        telemetry.addData("Current Stage", STAGE);
        telemetry.update();
    }

    private void initializeSensors() {
        // Distance sensor
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");

        // Odometry setup
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        //change the offsets to however far our odometry pods are from the dead center of the robot
        //x offset is for the side to side one, y offset is for the forward back one
        odo.setOffsets(-88, 0.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    private void resetOdometry() {
        odo.resetPosAndIMU();
        try { Thread.sleep(50); } catch (InterruptedException e) {}
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        try { Thread.sleep(50); } catch (InterruptedException e) {}
        odo.update();
    }

    private void moveToTargetStage(double targetY, int nextStage) {
        odo.update();
        Pose2D pos = odo.getPosition();

        double currentY = pos.getY(DistanceUnit.INCH);
        double error = targetY - currentY;
        double drivePower = error * KP;
        drivePower = Math.max(-0.3, Math.min(0.3, drivePower));

        double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);

        boolean targetReached = Math.abs(error) < 1.0;
        boolean obstacleClose = distanceInches < OBSTACLE_DISTANCE;

        if (targetReached || obstacleClose) {
            drive.stop();
            if (obstacleClose) {
                driveBackward(0.25, 900);
            }
            STAGE = nextStage;
            stageTimer.reset();
        } else {
            driveForward(drivePower);
        }

        telemetry.addData("Current Y (in)", "%.2f", currentY);
        telemetry.addData("Distance Sensor (in)", "%.2f", distanceInches);
    }


    private void turnToHeadingStage(double targetHeading, int nextStage) {
        odo.update();
        double currentHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);

        double error = targetHeading - currentHeading;
        error = ((error + 180) % 360) - 180;

        if (Math.abs(error) <= 1.0) {
            // Done turning
            drive.stop();
            STAGE = nextStage;
            stageTimer.reset();
        } else {
            double turnPower = 0.2 * Math.signum(error);  // same as before
            robot.fl_motor.setPower(-turnPower);
            robot.bl_motor.setPower(-turnPower);
            robot.fr_motor.setPower(turnPower);
            robot.br_motor.setPower(turnPower);
        }

        telemetry.addData("Target Heading", targetHeading);
        telemetry.addData("Current Heading", currentHeading);
        telemetry.addData("Turn Error", error);
    }


    private void launchArtifactsStage() {
        if (stageTimer.seconds() < 1.0) {
            shooter.startShootingFar();
        } else if (stageTimer.seconds() < 2.0) {
            artifactPusherArtifacts.startWheel();
        } else if (stageTimer.seconds() < 4.0) {
            intake.startPushing();
            artifactPusherArtifacts.startWheel();
        } else {
            shooter.stopShooting();
            intake.stopPushing();
            artifactPusherArtifacts.stopPushing();
            STAGE = 4;
            stageTimer.reset();
        }

        telemetry.addData("Launch Timer", "%.1f", stageTimer.seconds());
    }

    private void launchArtifactsStageFinal() {
        if (stageTimer.seconds() < 1.0) {
            shooter.startShootingFar();
        } else if (stageTimer.seconds() < 2.0) {
            artifactPusherArtifacts.startWheel();
        } else if (stageTimer.seconds() < 4.0) {
            intake.startPushing();
            artifactPusherArtifacts.startWheel();
        } else {
            shooter.stopShooting();
            intake.stopPushing();
            artifactPusherArtifacts.stopPushing();
            STAGE =14;
            stageTimer.reset();
        }

        telemetry.addData("Launch Timer", "%.1f", stageTimer.seconds());
    }

    private void intakeArtifactsStage() {
        if (stageTimer.seconds() < 0.0) {
            intake.startPushing();
        } else {
            intake.stopPushing();
            STAGE = 8;
            stageTimer.reset();
        }

        telemetry.addData("Intake Timer", "%.1f", stageTimer.seconds());
    }



    private void driveForward(double power) {
        robot.fl_motor.setPower(power);
        robot.fr_motor.setPower(power);
        robot.bl_motor.setPower(power);
        robot.br_motor.setPower(power);
    }

    private void driveBackward(double power, long timeMs) {
        robot.fl_motor.setPower(-power);
        robot.fr_motor.setPower(-power);
        robot.bl_motor.setPower(-power);
        robot.br_motor.setPower(-power);
        try { Thread.sleep(timeMs); } catch (InterruptedException e) {}
        drive.stop();
    }

    private void driveBackward(double power) {
        robot.fl_motor.setPower(-power);
        robot.fr_motor.setPower(-power);
        robot.bl_motor.setPower(-power);
        robot.br_motor.setPower(-power);


    }


//    private void resetOdometry(){
//              // Reset hardware
//        try {
//            odo.resetPosAndIMU();
//            Thread.sleep(50);
//            odo.setPosition(new Pose2D(...)); // Set new origin
//            Thread.sleep(50);               // Wait for processing
//            odo.update();  // Wait for processing
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//
//    }
}
