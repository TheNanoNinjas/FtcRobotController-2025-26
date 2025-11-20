package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.ArtifactPusher;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;

@Autonomous(name = "Auto Red Alliance Staged", group = "Competition")
public class AutoRedStaged extends OpMode {

    private final RobotHardware robot = new RobotHardware();
    private MecanumDrive drive;
    private Shooter shooter;
    private ArtifactPusher artifactPusherArtifacts;
    private Intaker intake;

    private GoBildaPinpointDriver odo;
    private Rev2mDistanceSensor distanceSensor;

    private static final double TARGET_Y_INCHES = 10.0;
    private static final double SECOND_TARGET_Y = 20;
    private static final double KP = 0.10;
    private static final double OBSTACLE_DISTANCE = 6.0;

    private static int STAGE = 1;
    private ElapsedTime stageTimer = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot);
        shooter = new Shooter(robot);
        artifactPusherArtifacts = new ArtifactPusher(robot);
        intake = new Intaker(robot);
        
        initializeSensors();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        stageTimer.reset();
        STAGE = 1;
    }

    @Override
    public void loop() {
        if (STAGE == 1) {
            moveToTargetStage();
        } else if (STAGE == 2) {
            turnToShootingAngle();
        } else if (STAGE == 3) {
            launchArtifactsStage();
        } else if (STAGE == 4) {
            turnToZeroStage();
        } else {
            drive.stop();
            shooter.stopShooting();
        }
        
        telemetry.addData("Current Stage", STAGE);
        telemetry.update();
    }

    private void initializeSensors() {
        // Distance sensor
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");

        // Odometry setup
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        //change the offsets to however far our odometry pods are from the dead center of the robot
        //x offset is for the side to side one, y offset is for the forward back one
        odo.setOffsets(-82.5, 0.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    private void moveToTargetStage() {
        odo.update();
        Pose2D pos = odo.getPosition();
        
        double currentY = pos.getY(DistanceUnit.INCH);
        double error = TARGET_Y_INCHES - currentY;
        double drivePower = error * KP;
        drivePower = Math.max(-0.3, Math.min(0.3, drivePower));
        
        double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);
        
        boolean targetReached = Math.abs(error) < 1.0;
        boolean obstacleClose = distanceInches < OBSTACLE_DISTANCE;
        
        if (targetReached || obstacleClose) {
            drive.stop();
            if (obstacleClose) {
                driveBackward(0.25, 900);
            }
            STAGE = 2;
            stageTimer.reset();
        } else {
            driveForward(drivePower);
        }
        
        telemetry.addData("Current Y (in)", "%.2f", currentY);
        telemetry.addData("Distance Sensor (in)", "%.2f", distanceInches);
    }
    
    private void turnToShootingAngle() {
        odo.update();
        double currentHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
        double error = 40 - currentHeading;
        error = ((error + 180) % 360) - 180;
        
        if (Math.abs(error) <= 1.0) {
            drive.stop();
            STAGE = 3;
            stageTimer.reset();
        } else {
            double turnPower = 0.2 * Math.signum(error);
            robot.fl_motor.setPower(-turnPower);
            robot.bl_motor.setPower(-turnPower);
            robot.fr_motor.setPower(turnPower);
            robot.br_motor.setPower(turnPower);
        }
        
        telemetry.addData("Target Heading", 40);
        telemetry.addData("Current Heading", currentHeading);
    }
    
    private void launchArtifactsStage() {
        if (stageTimer.seconds() < 1.0) {
            shooter.startShootingFar();
        } else if (stageTimer.seconds() < 7.0) {
            intake.startPushing();
            artifactPusherArtifacts.startWheel();
        } else {
            shooter.stopShooting();
            intake.stopPushing();
            artifactPusherArtifacts.stopPushing();
            STAGE = 4;
            stageTimer.reset();
        }
        
        telemetry.addData("Launch Timer", "%.1f", stageTimer.seconds());
    }
    
    private void turnToZeroStage() {
        odo.update();
        double currentHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
        double error = 0 - currentHeading;
        error = ((error + 180) % 360) - 180;
        
        if (Math.abs(error) <= 1.0) {
            drive.stop();
            STAGE = 5; // Complete
        } else {
            double turnPower = 0.2 * Math.signum(error);
            robot.fl_motor.setPower(-turnPower);
            robot.bl_motor.setPower(-turnPower);
            robot.fr_motor.setPower(turnPower);
            robot.br_motor.setPower(turnPower);
        }
        
        telemetry.addData("Final Turn Heading", currentHeading);
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
        try { Thread.sleep(timeMs); } catch (InterruptedException e) {}
        drive.stop();
    }
}
