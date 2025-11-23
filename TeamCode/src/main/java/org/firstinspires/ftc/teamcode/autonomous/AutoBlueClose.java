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

@Autonomous(name = "Auto Blue Alliance Close", group = "Competition")
public class AutoBlueClose extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();
    private MecanumDrive drive;
    private Shooter shooter;
    private ArtifactPusher artifactPusherArtifacts;
    private Intaker intake;

    private GoBildaPinpointDriver odo;
    private Rev2mDistanceSensor distanceSensor;

    private static final double TARGET_Y_INCHES = -60;
    private static final double INTAKE_TARGET_Y = -21;
    private static final double MOVE_INTAKE_FORWARDS_Y = 21;
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

        //shoot
        launchArtifacts();

        //turn
        turnToHeading(135);

        //strafe to intake
         double startX = odo.getPosition().getX(DistanceUnit.INCH);
        strafeToX(startX - 45, 0.3); // strafe left

        intakeArtifacts(5000);

        //go forward to intake artifacts
        moveToIntakeY();
        //go back after intaking
        moveForwardAfterIntake();

        startX = odo.getPosition().getX(DistanceUnit.INCH);
        strafeToX(startX + 45, 0.3);

        turnToHeading(0);

        launchArtifacts();

        drive.stop();
        shooter.stopShooting();
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

    private void strafeToX(double targetXInches, double basePower) {
        odo.update();
        Pose2D startPos = odo.getPosition();
        double startHeading = startPos.getHeading(AngleUnit.DEGREES);

        double currentX = startPos.getX(DistanceUnit.INCH);
        double error = targetXInches - currentX;
        double direction = Math.signum(error);

        telemetry.addLine("Strafe Started");
        telemetry.update();

        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            currentX = pos.getX(DistanceUnit.INCH);
            double currentHeading = pos.getHeading(AngleUnit.DEGREES);
            error = targetXInches - currentX;

            // Stop when close enough
            if (Math.abs(error) < 0.5) break;

            // Calculate heading error
            double headingError = ((startHeading - currentHeading + 180) % 360) - 180;


            double correction = 0.02 * headingError;  // tweak 0.02 if needed

            // Base strafe power
            double strafePower = direction * Math.abs(basePower);

            // Apply heading correction to each side
            double flPower = strafePower - correction;
            double blPower = -strafePower - correction;
            double frPower = -strafePower + correction;
            double brPower = strafePower + correction;

            robot.fl_motor.setPower(flPower);
            robot.bl_motor.setPower(blPower);
            robot.fr_motor.setPower(frPower);
            robot.br_motor.setPower(brPower);

            telemetry.addData("Target X (in)", targetXInches);
            telemetry.addData("Current X (in)", currentX);
            telemetry.addData("Remaining Distance (in)", "%.2f", error);
            telemetry.addData("Heading", "%.2f", currentHeading);
            telemetry.addData("Heading Error", "%.2f", headingError);
            telemetry.addData("Correction", "%.2f", correction);
            telemetry.update();

            sleep(20);
        }


        telemetry.addLine("=== Strafe Completed ===");
        telemetry.update();
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
    private void moveForwardAfterIntake() {
        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            double currentY = pos.getY(DistanceUnit.INCH);
            double error = MOVE_INTAKE_FORWARDS_Y - currentY;
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
