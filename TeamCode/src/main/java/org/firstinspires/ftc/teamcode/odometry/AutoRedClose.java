package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.ArtifactPusher;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagLimelight;

@Autonomous(name = "Auto Red Alliance Timed Close", group = "Competition")
public class AutoRedClose extends LinearOpMode {

    private RobotHardware robot;
    private MecanumDrive drive;
    private Shooter shooter;
    private ArtifactPusher pusher;
    private Intaker intake;
    private AprilTagLimelight tagLimelight;

    private static final double SHOOTER_MIN_RPM = 1550;
    private static final double SHOOTER_MAX_RPM = 1650;

    @Override
    public void runOpMode() {

        robot = new RobotHardware();
        robot.init(hardwareMap);

        tagLimelight = new AprilTagLimelight(hardwareMap);
        drive = new MecanumDrive(robot);
        shooter = new Shooter(robot, tagLimelight);
        pusher = new ArtifactPusher(robot, shooter);
        intake = new Intaker(robot);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;
driveBackwardTimed(0.35,1340);
launchArtifacts(4500);

turnLeftTimed(0.3,600);

driveBackwardTimed(0.35, 50);

turnLeftTimed(0.3,880);

        intake.startPushing();
        shooter.manualIntakeShooter();
        pusher.startArtifactPushing();

driveBackwardTimed(0.35,1850);

driveForwardTimed(0.35,1600);

        intake.stopPushing();
        shooter.stopShooting();
        pusher.stopPushing();

turnRightTimed(0.3,900);

driveForwardTimed(0.35,50);

turnRightTimed(0.3,500);

driveBackwardTimed(0.3,100);

        launchArtifacts(4500);

        turnLeftTimed(0.3,1000);

        driveBackwardTimed(0.35,1500);
        stopAll();
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


    private void launchArtifacts(long totalTimeMs) {

        shooter.startShootingAutoClose();

        while (opModeIsActive() && !shooter.isCloseShotReady()) {
            idle();
        }

        long startTime = System.currentTimeMillis();

        while (opModeIsActive() &&
                System.currentTimeMillis() - startTime < totalTimeMs) {

            pusher.startAutoPush();
            intake.startPushing();
            sleep(170);   // feed time

            pusher.stopPushing();
            intake.stopPushing();
            sleep(1300);
        }

        shooter.stopShooting();
    }






    private void driveForwardTimed(double power, long timeMs) {
        setDrivePower(power, power, power, power);
        sleep(timeMs);
        robot.stopAllMotors();
    }

    private void driveBackwardTimed(double power, long timeMs) {
        setDrivePower(-power, -power, -power, -power);
        sleep(timeMs);
        robot.stopAllMotors();
    }

    private void turnRightTimed(double power, long timeMs) {
        setDrivePower(power, -power, power, -power);
        sleep(timeMs);
        robot.stopAllMotors();
    }

    private void turnLeftTimed(double power, long timeMs) {
        setDrivePower(-power, power, -power, power);
        sleep(timeMs);
        robot.stopAllMotors();
    }

    private void setDrivePower(double fl, double fr, double bl, double br) {
        robot.fl_motor.setPower(fl);
        robot.fr_motor.setPower(fr);
        robot.bl_motor.setPower(bl);
        robot.br_motor.setPower(br);
    }

    private void stopAll() {
        robot.stopAllMotors();
        shooter.stopShooting();
        intake.stopPushing();
        pusher.stopPushing();
    }
}
