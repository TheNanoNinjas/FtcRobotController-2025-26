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


    private GoBildaPinpointDriver odo;
    private Rev2mDistanceSensor distanceSensor;


    @Override
    public void init() {
        robot.init(hardwareMap);

        drive = new MecanumDrive(robot);
        shooter = new Shooter(robot);
        intake = new Intaker(robot);
        artifactPusher = new ArtifactPusher(robot);

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
        handleIntake();
        handlePush();
        launchArtifacts();

        updateOdoMetrics();
        telemetry.addData("Status", "Running");
       // telemetry.update();
    }

    private void updateOdoMetrics() {


        odo.update();
        Pose2D pos = odo.getPosition();
        double currentY = pos.getY(DistanceUnit.INCH);
        double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);
        double currentHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
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
            // Robot-relative drive
            drive.mecanumDrive(-gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x);
        } else {
            // Field-relative drive
            drive.driveFieldRelative(-gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    robot.imu);
        }
    }

    private void handleIntake() {
        if (gamepad1.left_bumper) {
            intake.startPushing();
        } else {
            intake.stopPushing();
        }
    }


    private void handlePush() {
        if (gamepad2.cross) {
            artifactPusher.startWheel();
            intake.startPushing();
        } else if (gamepad2.right_stick_button) {

            intake.reversePush();
            artifactPusher.reversePush();

        }
        else if(gamepad1.left_bumper){
            intake.startPushing();


        }


        else {
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

