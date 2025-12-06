package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.autonomous.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.ArtifactPusher;

@TeleOp(name = "OpMode Test")
public class
OpModeTest extends OpMode {
    RobotHardware robot = new RobotHardware();
    MecanumDrive drive;
    Shooter shooter;
    Intaker intake;
    ArtifactPusher artifactPusher;


    @Override
    public void init() {
        robot.init(hardwareMap);

        drive = new MecanumDrive(robot);
        shooter = new Shooter(robot);
        intake = new Intaker(robot);
        artifactPusher = new ArtifactPusher(robot);


        robot.logHardwareStatus(telemetry);
        robot.displayPortMapping(telemetry);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData("Status", "OpMode Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        handleDriving();

        handlePush();
        launchArtifacts();

        updateOdoMetrics();
        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    private void updateOdoMetrics() {


        Pose2D pos = robot.getOdoPosition();
        double currentY = pos.getY(DistanceUnit.MM);
        double distanceInches = robot.getOdoPositionY(DistanceUnit.MM);
        double currentHeading = robot.getOdoHeading(AngleUnit.DEGREES);
        telemetry.addData("Intake Move Y (in)", "%.2f", currentY);
        telemetry.addData("Distance Sensor (in)", "%.2f", distanceInches);
        telemetry.addData("Current Heading (Degree)", "%.2f", currentHeading);

    }

    @Override
    public void stop() {
        robot.stopAllMotors();
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    private void handleDriving() {
        // Reset IMU heading with Cross button
        if (gamepad1.cross) {
            robot.imu.resetYaw();
        }

        // Field-relative drive (default) or robot-relative (right bumper)
        if (gamepad1.right_bumper) {

            // Field-relative drive
            drive.driveFieldRelative(-gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    robot.imu);

        } else {


            // Robot-relative drive
            drive.mecanumDrive(-gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x);
        }
    }


    private void handlePush() {
        if (gamepad2.cross) {
            artifactPusher.startWheel();
            intake.startPushing();
        } else if (gamepad2.right_stick_button) {

            intake.reversePush();
            artifactPusher.reversePush();

        } else if (gamepad2.right_trigger > 0)
        {
            artifactPusher.startWheel();

        } else if (gamepad2.left_trigger > 0)
        {
            artifactPusher.reversePush();
        } else if (gamepad1.left_bumper) {
            intake.startPushing();

        } else if (gamepad2.dpad_up) {
            intake.startPushing();

        } else {
            artifactPusher.stopPushing();
            intake.stopPushing();
        }
    }

    private void launchArtifacts() {
        // Right bumper for close range shooting
        if (gamepad2.right_bumper) {
            shooter.startShootingClose();
        }
        // Left bumper for far range shooting
        else if (gamepad2.left_bumper) {
            shooter.startShootingFar();
        } else if (gamepad2.square) {
            shooter.manualIntakeShooter();

        } else {
            shooter.stopShooting();
        }
    }

}

