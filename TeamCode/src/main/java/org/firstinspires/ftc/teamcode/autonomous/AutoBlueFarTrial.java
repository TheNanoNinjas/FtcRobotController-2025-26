package org.firstinspires.ftc.teamcode.autonomous;

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

@Autonomous(name = "Auto Red Alliance Far", group = "Competition")

public class AutoBlueFarTrial extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();
    private MecanumDrive drive;
    private Shooter shooter;
    private ArtifactPusher artifactPusherArtifacts;
    private Intaker intake;

   // private GoBildaPinpointDriver odo;
   // private Rev2mDistanceSensor distanceSensor;

    private static final double KP = 0.05;

    @Override
    public void runOpMode() {
        // Initialize hardware and mechanisms
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot);
        shooter = new Shooter(robot);
        artifactPusherArtifacts = new ArtifactPusher(robot);
        intake = new Intaker(robot);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            executeAutonomousSequence();
        }
    }

    private void executeAutonomousSequence() {
        telemetry.addLine("Starting autonomous sequence");
        telemetry.update();

        // Move forward to shooting position
        moveToYTarget(192);

        // Turn to shooting angle
        turnToHeading(19);

        // Launch artifacts
        launchArtifacts();

        //turn back to 0, last value was 3
        turnToHeading(15);

        // go forward
        moveToYTarget(810);

        //turn to intake earlier value was 100
        turnToHeading(287);

        sleep(500);

        //start intaking
        startIntake();

        //go forward to intake, earlier value was 810
      //  moveBackwardsToYTarget(800);
        driveBackwardTimed(0.3,1750);
startIntake();
       sleep(750);

        //move backwards after intake, earlier time was 2350
     driveForwardtimed(0.3,1600);
        stopIntake();

        //turn back to 0, earlier value is 4
        turnToHeading(40);

        //move backwards to shooting zone
        driveBackwardTimed(0.25, 1600);
        sleep(1000);

        moveToYTarget(500);
        //turn to shooting angle
            turnToHeading(180);

        //launch artifacts
        launchArtifacts();

        // double startX = odo.getPosition().getX(DistanceUnit.MM);
        //strafeToX(startX + 5.0, 0.3); // strafe right

        driveForwardtimed(0.2, 3000);

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

            if (error < 2) break;
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

            if (error < 2) break;
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

    private void launchArtifacts() {
        telemetry.addLine("Starting shooter motors");
        telemetry.update();
        shooter.startShootingFar();
        sleep(1000);

        telemetry.addLine("Starting artifact pusher wheel");
        telemetry.update();
        artifactPusherArtifacts.startWheel();
        sleep(1000);

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
