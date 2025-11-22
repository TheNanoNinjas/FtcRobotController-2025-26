package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.ArtifactPusher;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;

@Autonomous(name = "Auto Blue Alliance", group = "Competition")
public class AutoBlue extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();
    private MecanumDrive drive;
    private Shooter shooter;
    private ArtifactPusher artifactPusherArtifacts;
    private Intaker intake;

    private GoBildaPinpointDriver odo;
    private Rev2mDistanceSensor distanceSensor;

    private static final double TARGET_Y_INCHES = 10.0;
    private static final double SECOND_TARGET_Y = 21;
    private static final double INTAKE_TARGET_Y = -32;
    private static final double MOVE_INTAKE_BACKWARDS_Y = -32;
    private static final double MOVE_BACKWARDS_Y = -21;
    private static final double KP = 0.10;
    private static final double OBSTACLE_DISTANCE = 6.0;

    @Override
    public void runOpMode() {
        // Initialize hardware and mechanisms
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot);
        shooter = new Shooter(robot);
        artifactPusherArtifacts = new ArtifactPusher(robot);
        intake = new Intaker(robot);


        initializeSensors();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            executeAutonomousSequence();
        }
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
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    private void executeAutonomousSequence() {
        // Move forward to target or until obstacle detected
        moveToTarget();

        // Turn to shooting angle
        turnToHeading(22);

        // Launch artifacts
        launchArtifacts();

        turnToHeading(0);

        moveToY();

        turnToHeading(270);

        intakeArtifacts(5000);
        moveToIntakeY();

        moveBackwardsIntake();

        turnToHeading(0);

        moveBackwardsShoot();

        turnToHeading(22);

        launchArtifacts();

        drive.stop();
        shooter.stopShooting();
    }

    private void moveToTarget() {
        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            double currentY = pos.getY(DistanceUnit.INCH);
            double error = TARGET_Y_INCHES - currentY;
            double drivePower = error * KP;
            drivePower = Math.max(-0.4, Math.min(0.4, drivePower));

            double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);

            boolean targetReached = Math.abs(error) < 1.0;
            boolean obstacleClose = distanceInches < OBSTACLE_DISTANCE;

            if (targetReached || obstacleClose) {
                drive.stop();
                telemetry.addLine("Movement stopped!");

                if (targetReached) {
                    telemetry.addLine("Reason: Target reached");
                }
                if (obstacleClose) {
                    telemetry.addLine("Reason: Obstacle detected");
                    telemetry.addLine("Backing up...");
                    telemetry.update();

                    // Back up from obstacle
                    driveBackward(0.25, 900);
                }
                telemetry.update();
                break;
            }


            driveForward(drivePower);

            telemetry.addData("Current Y (in)", "%.2f", currentY);
            telemetry.addData("Distance Sensor (in)", "%.2f", distanceInches);
            telemetry.addData("Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }


        drive.stop();
    }

    private void moveToY() {
        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            double currentY = pos.getY(DistanceUnit.INCH);
            double error = SECOND_TARGET_Y - currentY;
            double drivePower = error * KP;
            drivePower = Math.max(-0.4, Math.min(0.4, drivePower));

            double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);

            boolean targetReached = Math.abs(error) < 1.0;
            boolean obstacleClose = distanceInches < OBSTACLE_DISTANCE;

            if (targetReached || obstacleClose) {
                drive.stop();
                telemetry.addLine("Movement stopped!");

                if (targetReached) {
                    telemetry.addLine("Reason: Target reached");
                }
                if (obstacleClose) {
                    telemetry.addLine("Reason: Obstacle detected");
                    telemetry.addLine("Backing up...");
                    telemetry.update();

                    // Back up from obstacle
                    driveBackward(0.25, 900);
                }
                telemetry.update();
                break;
            }


            driveForward(drivePower);

            telemetry.addData("Current Y (in)", "%.2f", currentY);
            telemetry.addData("Distance Sensor (in)", "%.2f", distanceInches);
            telemetry.addData("Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }


        drive.stop();
    }
    private void moveToIntakeY() {
        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            double currentY = pos.getY(DistanceUnit.INCH);
            double error = INTAKE_TARGET_Y - currentY;
            double drivePower = error * KP;
            drivePower = Math.max(-0.4, Math.min(0.4, drivePower));

            double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);

            boolean targetReached = Math.abs(error) < 1.0;
            boolean obstacleClose = distanceInches < OBSTACLE_DISTANCE;

            if (targetReached || obstacleClose) {
                drive.stop();
                telemetry.addLine("Movement stopped!");

                if (targetReached) {
                    telemetry.addLine("Reason: Target reached");
                }
                if (obstacleClose) {
                    telemetry.addLine("Reason: Obstacle detected");
                    telemetry.addLine("Backing up...");
                    telemetry.update();

                    // Back up from obstacle
                    driveBackward(0.25, 900);
                }
                telemetry.update();
                break;
            }


            driveForward(drivePower);

            telemetry.addData("Current Y (in)", "%.2f", currentY);
            telemetry.addData("Distance Sensor (in)", "%.2f", distanceInches);
            telemetry.addData("Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }


        drive.stop();
    }


    private void moveBackwardsIntake() {
        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            double currentY = pos.getY(DistanceUnit.INCH);
            double error = MOVE_INTAKE_BACKWARDS_Y - currentY;
            double drivePower = error * KP;
            drivePower = Math.max(-0.4, Math.min(0.4, drivePower));

            double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);

            boolean targetReached = Math.abs(error) < 1.0;
            boolean obstacleClose = distanceInches < OBSTACLE_DISTANCE;

            if (targetReached || obstacleClose) {
                drive.stop();
                telemetry.addLine("Movement stopped!");

                if (targetReached) {
                    telemetry.addLine("Reason: Target reached");
                }
                if (obstacleClose) {
                    telemetry.addLine("Reason: Obstacle detected");
                    telemetry.addLine("Backing up...");
                    telemetry.update();

                    // Back up from obstacle
                    driveBackward(0.25, 900);
                }
                telemetry.update();
                break;
            }


            driveForward(drivePower);

            telemetry.addData("Current Y (in)", "%.2f", currentY);
            telemetry.addData("Distance Sensor (in)", "%.2f", distanceInches);
            telemetry.addData("Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }


        drive.stop();
    }
    private void moveBackwardsShoot() {
        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            double currentY = pos.getY(DistanceUnit.INCH);
            double error = MOVE_BACKWARDS_Y - currentY;
            double drivePower = error * KP;
            drivePower = Math.max(-0.4, Math.min(0.4, drivePower));

            double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);

            boolean targetReached = Math.abs(error) < 1.0;
            boolean obstacleClose = distanceInches < OBSTACLE_DISTANCE;

            if (targetReached || obstacleClose) {
                drive.stop();
                telemetry.addLine("Movement stopped!");

                if (targetReached) {
                    telemetry.addLine("Reason: Target reached");
                }
                if (obstacleClose) {
                    telemetry.addLine("Reason: Obstacle detected");
                    telemetry.addLine("Backing up...");
                    telemetry.update();

                    // Back up from obstacle
                    driveBackward(0.25, 900);
                }
                telemetry.update();
                break;
            }


            driveForward(drivePower);

            telemetry.addData("Current Y (in)", "%.2f", currentY);
            telemetry.addData("Distance Sensor (in)", "%.2f", distanceInches);
            telemetry.addData("Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }


        drive.stop();
    }
    private void turnToHeading(double targetHeading) {
        odo.update();
        double currentHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
        double error = targetHeading - currentHeading;

        // Normalize error to range -180 to +180
        error = ((error + 180) % 360) - 180;

        while (opModeIsActive() && Math.abs(error) > 1.0) {
            odo.update();
            currentHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
            error = targetHeading - currentHeading;
            error = ((error + 180) % 360) - 180;

            double turnPower = 0.3 * Math.signum(error);

            // Turn using drive motors
            robot.fl_motor.setPower(-turnPower);
            robot.bl_motor.setPower(-turnPower);
            robot.fr_motor.setPower(turnPower);
            robot.br_motor.setPower(turnPower);

            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        drive.stop();
    }

    private void launchArtifacts() {
        // Start shooter motors
        shooter.startShootingFar();
        sleep(1000);
        artifactPusherArtifacts.startWheel();
        sleep(1000);

        intake.startPushing();
        artifactPusherArtifacts.startWheel();
        sleep(1500);

        // Stop shooter
        shooter.stopShooting();
        intake.stopPushing();
        artifactPusherArtifacts.stopPushing();

        telemetry.addLine("Artifact launch completed");
        telemetry.update();
    }

    private void intakeArtifacts(long timeMs) {
        intake.startPushing();
        sleep(timeMs);
        intake.stopPushing();
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
        sleep(timeMs);
        drive.stop();
    }
}
