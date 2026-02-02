package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagLimelight;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.ArtifactPusher;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;

@Autonomous(name = "Auto Blue Alliance Timed Far", group = "Competition")
@Disabled
public class AutoBlueFar extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();
    private MecanumDrive drive;
    private Shooter shooter;
    private ArtifactPusher artifactPusherArtifacts;
    private Intaker intake;
    private AprilTagLimelight tagLimelight;

    // private GoBildaPinpointDriver odo;
    // private Rev2mDistanceSensor distanceSensor;

    private static final double KP = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware and mechanisms
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot);
        shooter = new Shooter(robot, tagLimelight);
        artifactPusherArtifacts = new ArtifactPusher(robot,shooter);
        intake = new Intaker(robot);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            executeAutonomousSequence();
        }
    }

    private void executeAutonomousSequence() throws InterruptedException {
        telemetry.addLine("Starting autonomous sequence");
        telemetry.update();

        // Move forward to shooting position
        driveForwardtimed(0.3,400);

        // Turn to shooting angle
        turnLeft(0.3,190);

        // Launch artifacts
        launchArtifacts();

        //turn back to 0, last value was 395
        turnRight(0.3,320);

        // go forward
        driveForwardtimed(0.3,1500);

        //turn to intake earlier value was 100
        turnRight(0.3,900);

        sleep(500);

        //start intaking
        startIntake();

        //go forward to intake, earlier value was 810
        driveBackwardTimed(0.3,2350);
        startIntake();
        //move backwards after intake, earlier time was 2350
        driveForwardtimed(0.3, 2050);

        stopIntake();

        //turn back to 0, earlier value is 4
        turnLeft(0.3,1000);

        //move backwards to shooting zone
        driveBackwardTimed(0.25, 1500);
        sleep(1000);

        driveForwardtimed(0.3,300);

        //  turn to shooting angle
        turnLeft(0.3,190);

        //launch artifacts
        launchArtifacts();

        // double startX = odo.getPosition().getX(DistanceUnit.MM);
        //strafeToX(startX + 5.0, 0.3); // strafe right

        driveForwardtimed(0.3, 2000);

        telemetry.addLine("Autonomous sequence complete");
        telemetry.update();
        drive.stop();
        shooter.stopShooting();
        intake.stopPushing();
    }

    private void moveToYTarget(double targetY) {
        long startTime = System.currentTimeMillis();
        long timeout = 6500;  // timeout in milliseconds)

        while (opModeIsActive()) {

        /*     if (robot.getOdoPositionY(DistanceUnit.MM)> targetY &&
                     System.currentTimeMillis() - startTime > timeout){
             }*/

            double y = robot.getOdoPositionY(DistanceUnit.MM);
            double error = targetY - y;

            double drivePower = 0.25;
            if (Math.abs(error) < 6) drivePower = 0.25 + (error * KP);

            drivePower = Math.max(-0.3, Math.min(0.3, drivePower));

            driveForward(drivePower);

            telemetry.addData("Target Y", targetY);
            telemetry.addData("Current Y", y);
            telemetry.addData("Error", error);
            telemetry.addData("Drive Power", drivePower);
            telemetry.addData("Timeout", (System.currentTimeMillis() - startTime) + " / " + timeout);
            telemetry.update();

            if (error < 0) break;
        }

        drive.stop();
        telemetry.addLine("Y move finished");
        telemetry.update();
    }


    private void moveBackwardsToYTarget(double targetY) {
        long startTime = System.currentTimeMillis();
        long timeout = 5000;  // timeout in milliseconds)

        while (opModeIsActive()) {

            // timeout
            if (System.currentTimeMillis() - startTime > timeout) {
                telemetry.addLine("Timeout: moving on to next step");
                telemetry.update();
                break;
            }


            double y = robot.getOdoPositionY(DistanceUnit.MM);
            double error = targetY - y;

            double drivePower = 0.35;
            if (Math.abs(error) < 6) drivePower = 0.25 + (error * KP);

            drivePower = Math.max(-0.35, Math.min(0.35, drivePower));

            driveBackward(drivePower);

            telemetry.addData("Target Y", targetY);
            telemetry.addData("Current Y", y);
            telemetry.addData("Error", error);
            telemetry.addData("Drive Power", drivePower);
            telemetry.update();

            if (error < 0) break;
        }
        drive.stop();
        telemetry.addLine("Y target reached");
        telemetry.update();
    }

    private void turnToHeading(double targetHeading) {

        double currentHeading = robot.getOdoHeading(AngleUnit.DEGREES);
        double error = targetHeading - currentHeading;

        // Normalize error to range -180 to +180
        error = ((error + 180) % 360) - 180;

        while (opModeIsActive() && Math.abs(error) > 1.0) {

            currentHeading = robot.getOdoHeading(AngleUnit.DEGREES);
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

    private void strafeToX(double targetXMM, double basePower) {

        Pose2D startPos = robot.getOdoPosition();
        double startHeading = startPos.getHeading(AngleUnit.DEGREES);

        double currentX = startPos.getX(DistanceUnit.MM);
        double error = targetXMM - currentX;
        double direction = Math.signum(error);

        telemetry.addLine("Strafe Started");
        telemetry.update();

        while (opModeIsActive()) {

            Pose2D pos = robot.getOdoPosition();

            currentX = pos.getX(DistanceUnit.MM);
            double currentHeading = pos.getHeading(AngleUnit.DEGREES);
            error = targetXMM - currentX;

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

            robot.fl_motor.setPower(-flPower);
            robot.bl_motor.setPower(blPower);
            robot.fr_motor.setPower(frPower);
            robot.br_motor.setPower(-brPower);

            telemetry.addData("Target X (in)", targetXMM);
            telemetry.addData("Current X (in)", currentX);
            telemetry.addData("Remaining Distance (in)", "%.2f", error);
            telemetry.addData("Heading", "%.2f", currentHeading);
            telemetry.addData("Heading Error", "%.2f", headingError);
            telemetry.addData("Correction", "%.2f", correction);
            telemetry.update();

            sleep(20);
        }

        robot.stopAllMotors();
        telemetry.addLine("Strafe Completed");
        telemetry.update();
    }

    private void launchArtifacts() throws InterruptedException {
        telemetry.addLine("Starting shooter motors");
        telemetry.update();
        shooter.startShootingAutoFar();
        sleep(1500);

        telemetry.addLine("Starting artifact pusher wheel");
        telemetry.update();
     //   artifactPusherArtifacts.startWheel();
        sleep(1000);

        telemetry.addLine("Starting intake and pusher");
        telemetry.update();
        intake.startPushingAuto(750);
        sleep(500);

        intake.stopPushing();
        artifactPusherArtifacts.stopPushing();
        sleep(1500);

        intake.startPushing();
      //  artifactPusherArtifacts.startWheel();
        sleep(1100);

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
    private void turnRight(double power, long timeMs) {
        robot.fr_motor.setPower(-power);
        robot.br_motor.setPower(-power);
        robot.fl_motor.setPower(power);
        robot.bl_motor.setPower(power);

        sleep(timeMs);
        robot.stopAllMotors();
    }

    private void turnLeft(double power, long timeMs) {
        robot.fr_motor.setPower(power);
        robot.br_motor.setPower(power);
        robot.fl_motor.setPower(-power);
        robot.bl_motor.setPower(-power);

        sleep(timeMs);
        robot.stopAllMotors();
    }

}
