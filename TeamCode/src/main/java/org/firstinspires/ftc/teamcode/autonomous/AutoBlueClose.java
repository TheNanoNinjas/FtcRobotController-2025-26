package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

    private static final double KP = 0.05;

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
        odo.setOffsets(-88, 0.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    private void executeAutonomousSequence() {
        telemetry.addLine("Starting autonomous sequence");
        telemetry.update();

        // Move forward to shooting position
        driveBackwardTimed(0.3,2800);

        // Launch artifacts
        launchArtifacts();

     //   driveForwardtimed(0.3,600);

        turnToHeading(225);

        startIntake();

        // go forward
        moveBackwardsToYTarget(-35);

        sleep(1000);
        driveForwardtimed(0.3,1950);

        //start intaking
        stopIntake();

        //go forward to intake
        //   odo.update();
        //   startX = odo.getPosition().getX(DistanceUnit.INCH);
        //   strafeToX(startX + 6, 0.3); // strafe right

        turnToHeading(347);

        launchArtifacts();

        turnToHeading(297);

        driveBackwardTimed(0.3,1000);

        telemetry.addLine("Autonomous sequence complete");
        telemetry.update();
        drive.stop();
        shooter.stopShooting();
        intake.stopPushing();
    }

    private void moveToYTarget(double targetY) {
        long startTime = System.currentTimeMillis();
        long timeout = 6500;  // timeout in milliseconds)
        double distINCH = robot.distanceSensor.getDistance(DistanceUnit.INCH);
        while (opModeIsActive()) {

            // timeout
            if (System.currentTimeMillis() - startTime > timeout) {
                telemetry.addLine("TIMEOUT: going to next operation");
                telemetry.update();
                break;
            }


            odo.update();
            double y = odo.getPosition().getY(DistanceUnit.INCH);
            double error = targetY - y;

            double drivePower = 0.25;
            if (Math.abs(error) < 6) drivePower = 0.25 + (error * KP);

            drivePower = Math.max(-0.3, Math.min(0.3, drivePower));

            driveForward(drivePower);

            boolean targetReached = Math.abs(error) < 1.0;
            boolean obstacleClose = distINCH < 20;

            if (targetReached || obstacleClose) {
                robot.stopAllMotors();
                telemetry.addLine("STOPPED!");
                if (targetReached) telemetry.addLine("Reason: Target Reached");
                if (obstacleClose) {
                    telemetry.addLine("Reason: Obstacle Detected");
                    telemetry.update();

                    robot.stopAllMotors();
                }
                telemetry.update();
                break;
            }
            telemetry.addData("Target Y", "%.2f", targetY);
            telemetry.addData("Current Y", "%.2f", y);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Drive Power", "%.2f", drivePower);
            telemetry.update();

            if (error< 0) break;
        }
        drive.stop();
        telemetry.addLine("Y target reached");
        telemetry.update();
    }

    private void moveBackwardsToYTarget(double targetY) {
        long startTime = System.currentTimeMillis();
        long timeout = 4500;  // timeout in milliseconds)
        double distCM = robot.distanceSensor.getDistance(DistanceUnit.INCH);

        while (opModeIsActive()) {

            // timeout
            if (System.currentTimeMillis() - startTime > timeout) {
                telemetry.addLine("Timeout: moving on to next step");
                telemetry.update();
                break;
            }

            odo.update();
            double y = odo.getPosition().getY(DistanceUnit.INCH);
            double error = targetY - y;

            double drivePower = 0.25;
            if (Math.abs(error) < 6) drivePower = 0.25 + (error * KP);

            drivePower = Math.max(-0.3, Math.min(0.3, drivePower));

            driveBackward(drivePower);

            boolean targetReached = Math.abs(error) < 1.0;
            boolean obstacleClose = distCM < 40;

            if (targetReached || obstacleClose) {
                telemetry.addLine("STOPPED!");
                if (targetReached) telemetry.addLine("Reason: Target Reached");
                if (obstacleClose) {
                    telemetry.addLine("Reason: Obstacle Detected");
                    telemetry.update();

                    robot.stopAllMotors();
                }
                telemetry.update();
                break;
            }
            telemetry.addData("Target Y", "%.2f", targetY);
            telemetry.addData("Current Y", "%.2f", y);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Drive Power", "%.2f", drivePower);
            telemetry.update();

            if (error < 0) break;
        }
        drive.stop();
        telemetry.addLine("Y target reached");
        telemetry.update();
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

            double turnPower = 0.25 * Math.signum(error);

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

    private void strafeToX(double targetXInches, double power) {
        final double HEADING_KP = 0.015;   // heading correction
        final double POSITION_TOLERANCE = 0.5;  // inches

        odo.update();
        Pose2D startPos = odo.getPosition();
        double startHeading = startPos.getHeading(AngleUnit.DEGREES);

        telemetry.addLine("Strafe Started");
        telemetry.update();

        while (opModeIsActive()) {

            odo.update();
            Pose2D pos = odo.getPosition();

            double currentX = pos.getX(DistanceUnit.INCH);
            double heading = pos.getHeading(AngleUnit.DEGREES);

            double errorX = targetXInches - currentX;

            // STOP CONDITION
            if (errorX < POSITION_TOLERANCE) break;

            // Determine left or right
            double strafePower = Math.copySign(power, errorX);

            // Heading correction (keeps robot facing same direction)
            double headingError = ((startHeading - heading + 180) % 360) - 180;
            double correction = HEADING_KP * headingError;

            // Proper mecanum strafe powers
            double fl =  strafePower - correction;
            double fr = -strafePower + correction;
            double bl = -strafePower - correction;
            double br =  strafePower + correction;

            // Apply to motors
            robot.fl_motor.setPower(fl);
            robot.fr_motor.setPower(fr);
            robot.bl_motor.setPower(bl);
            robot.br_motor.setPower(br);

            // Telemetry
            telemetry.addData("Target X", targetXInches);
            telemetry.addData("Current X", currentX);
            telemetry.addData("Error X", errorX);
            telemetry.addData("Heading", heading);
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("Correction", correction);
            telemetry.update();

            sleep(15);
        }

        robot.stopAllMotors();
        telemetry.addLine("Strafe Completed");
        telemetry.update();
    }


    private void launchArtifacts() {
        telemetry.addLine("Starting shooter motors");
        telemetry.update();
        shooter.startShootingClose();
        sleep(1000);

        telemetry.addLine("Starting artifact pusher wheel");
        telemetry.update();
        artifactPusherArtifacts.startWheel();
        sleep(2500);

        telemetry.addLine("Starting intake and pusher");
        telemetry.update();
        intake.startPushing();
        artifactPusherArtifacts.startWheel();
        sleep(1500);

        telemetry.addLine("Stopping all launch mechanisms");
        telemetry.update();
        shooter.stopShooting();
        intake.stopPushing();
        artifactPusherArtifacts.stopPushing();

        telemetry.addLine("Artifact launch completed");
        telemetry.update();
    }

    private void startIntake() {
        intake.startPushing();
    }

    private void stopIntake() {
        intake.stopPushing();

    }

    private void driveForward(double power) {
        robot.fl_motor.setPower(power);
        robot.fr_motor.setPower(power);
        robot.bl_motor.setPower(power);
        robot.br_motor.setPower(power);
    }

    private void driveForwardtimed(double power, long timeMS) {
        robot.fl_motor.setPower(power);
        robot.fr_motor.setPower(power);
        robot.bl_motor.setPower(power);
        robot.br_motor.setPower(power);
        sleep(timeMS);
        robot.stopAllMotors();
    }

    private void driveBackwardTimed(double power, long timeMS) {
        robot.fl_motor.setPower(-power);
        robot.fr_motor.setPower(-power);
        robot.bl_motor.setPower(-power);
        robot.br_motor.setPower(-power);
        sleep(timeMS);
        robot.stopAllMotors();
    }

    private void driveBackward(double power) {
        robot.fl_motor.setPower(-power);
        robot.fr_motor.setPower(-power);
        robot.bl_motor.setPower(-power);
        robot.br_motor.setPower(-power);

    }
}
