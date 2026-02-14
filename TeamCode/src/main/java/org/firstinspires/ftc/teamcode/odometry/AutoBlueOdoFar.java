package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.ArtifactPusher;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagLimelight;

@Autonomous(name = "Auto Blue Alliance Far", group = "Competition")
public class AutoBlueOdoFar extends LinearOpMode {

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


        driveForwardTimed(0.3, 450);
        turnLeftTimed(0.3, 180);

        launchArtifacts(5000);

        turnRightTimed(0.3, 220);
        driveForwardTimed(0.35, 525);

        turnRightTimed(0.3, 883);
        sleep(500);

        intake.startPushing();
        shooter.manualIntakeShooter();
        pusher.startArtifactPushing();

        driveBackwardTimed(0.35, 2000);

        driveForwardTimed(0.3, 1700);

        turnLeftTimed(0.3, 900);

        intake.stopPushing();
        pusher.stopPushing();
        shooter.manualIntakeShooter();

        driveBackwardTimed(0.35, 900);
        sleep(1000);
        driveForwardTimed(0.3, 275);

        turnLeftTimed(0.3, 170);
        launchArtifacts(5000);

        driveForwardTimed(0.4, 1000);


        stopAll();
    }



    private void launchArtifacts(long totalTimeMs) {

        shooter.startShootingAutoFar();

        while (opModeIsActive() && !shooter.isFarShotReady()) {
            idle();
        }

        long startTime = System.currentTimeMillis();

        while (opModeIsActive() &&
                System.currentTimeMillis() - startTime < totalTimeMs) {

            pusher.startArtifactPushing();
            intake.startPushing();
                sleep(120);   // feed time

            pusher.stopPushing();
            intake.stopPushing();
            sleep(1350);
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
