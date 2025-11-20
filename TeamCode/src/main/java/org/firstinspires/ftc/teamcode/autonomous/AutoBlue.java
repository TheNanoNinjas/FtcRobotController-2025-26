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
import org.firstinspires.ftc.teamcode.mechanisms.Gate;

@Autonomous(name="Auto Blue Alliance", group="Competition")
public class AutoBlue extends LinearOpMode {
    
    // Hardware and mechanisms
    private RobotHardware robot = new RobotHardware();
    private MecanumDrive drive;
    private Shooter shooter;
    private Gate gate;
    
    // Sensors
    private GoBildaPinpointDriver odo;
    private Rev2mDistanceSensor distanceSensor;
    
    // Constants
    private static final double TARGET_Y_INCHES = 103.0;
    private static final double KP = 0.10;
    private static final double OBSTACLE_DISTANCE = 6.0;
    private static final double TARGET_HEADING = 47.0;
    
    @Override
    public void runOpMode() {
        // Initialize hardware and mechanisms
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot);
        shooter = new Shooter(robot);
        gate = new Gate(robot);
        
        // Initialize sensors
        initializeSensors();
        
        // Initialize gate position
        gate.setGatePosition(0.8);
        
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
        odo.setOffsets(-82.5, 0.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }
    
    private void executeAutonomousSequence() {
        // 1. Move forward to target or until obstacle detected
        moveToTarget();
        
        // 2. Turn to shooting angle
        turnToHeading(TARGET_HEADING);
        
        // 3. Launch artifacts
        launchArtifacts();
        
        // 4. Stop all systems
        drive.stop();
        shooter.stopShooting();
    }
    
    private void moveToTarget() {
        while (opModeIsActive()) {
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
            
            // Drive forward with calculated power
            driveForward(drivePower);
            
            // Telemetry
            telemetry.addData("Current Y (in)", "%.2f", currentY);
            telemetry.addData("Distance Sensor (in)", "%.2f", distanceInches);
            telemetry.addData("Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
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
            
            double turnPower = 0.2 * Math.signum(error);
            
            // Turn using drive subsystem
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
        // Start shooter motors
        shooter.startShooting();
        sleep(300);
        
        // Cycle gate 3 times to launch artifacts
        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            gate.setGatePosition(0.8);
            sleep(800);
            gate.setGatePosition(0.6);
            sleep(1000);
        }
        
        // Stop shooter
        shooter.stopShooting();
        
        telemetry.addData("Gate Position", gate.getGatePosition());
        telemetry.addLine("Artifact launch completed");
        telemetry.update();
    }
    
    // Helper methods for basic movement
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