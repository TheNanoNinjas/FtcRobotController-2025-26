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

@Autonomous(name = "Auto Red Alliance Far", group = "Competition")
public class AutoRedFar extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();
    private MecanumDrive drive;
    private Shooter shooter;
    private ArtifactPusher artifactPusherArtifacts;
    private Intaker intake;

    private GoBildaPinpointDriver odo;
    private Rev2mDistanceSensor distanceSensor;

    private static final double KP = 0.10;

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
        telemetry.addLine("Starting autonomous sequence");
        telemetry.update();
        
        // Move forward to shooting position
        telemetry.addLine("Moving to shooting position (Y=10)");
        telemetry.update();
        moveToYTarget(10);

        // Turn to shooting angle
        telemetry.addLine("Turning to shooting angle (338°)");
        telemetry.update();
        turnToHeading(338);

        // Launch artifacts
        telemetry.addLine("Launching first set of artifacts");
        telemetry.update();
        launchArtifacts();

        //turn back to 0
        telemetry.addLine("Turning to 0°");
        telemetry.update();
        turnToHeading(0);

        // go forward
        telemetry.addLine("Moving forward (Y=21)");
        telemetry.update();
        moveToYTarget(21);

        //turn to intake
        telemetry.addLine("Turning to intake position (90°)");
        telemetry.update();
        turnToHeading(90);

        //start intaking
        telemetry.addLine("Starting intake sequence");
        telemetry.update();
        intakeArtifacts(5000);

        //go forward to intake
        telemetry.addLine("Moving to intake zone (Y=-32)");
        telemetry.update();
        moveToYTarget(32);

        //move backwards after intake
        telemetry.addLine("Moving back from intake (Y=32)");
        telemetry.update();
        moveToYTarget(-32);

        //turn back to 0
        telemetry.addLine("Turning back to 0°");
        telemetry.update();
        turnToHeading(0);

        //move backwards to shooting zone
        telemetry.addLine("Moving to second shooting position (Y=-21)");
        telemetry.update();
        moveToYTarget(-21);

        //turn to shooting angle
        telemetry.addLine("Turning to final shooting angle (338°)");
        telemetry.update();
        turnToHeading(338);

        //launch artifacts
        telemetry.addLine("Launching second set of artifacts");
        telemetry.update();
        launchArtifacts();

        telemetry.addLine("Autonomous sequence complete");
        telemetry.update();
        drive.stop();
        shooter.stopShooting();
    }

    private void moveToYTarget(double targetY) {
        while (opModeIsActive()) {
            odo.update();
            double y = odo.getPosition().getY(DistanceUnit.INCH);
            double error = targetY - y;

            double drivePower = 0.45;
            if (Math.abs(error) < 6) drivePower = 0.25 + (error * KP);

            drivePower = Math.max(-0.5, Math.min(0.5, drivePower));

            driveForward(drivePower);

            telemetry.addData("Target Y", "%.2f", targetY);
            telemetry.addData("Current Y", "%.2f", y);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Drive Power", "%.2f", drivePower);
            telemetry.update();

            if (Math.abs(error) < 0.5) break;
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

    private void intakeArtifacts(long timeMs) {
        telemetry.addData("Starting intake for", "%d ms", timeMs);
        telemetry.update();
        intake.startPushing();
        sleep(timeMs);
        intake.stopPushing();
        telemetry.addLine("Intake sequence completed");
        telemetry.update();
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
