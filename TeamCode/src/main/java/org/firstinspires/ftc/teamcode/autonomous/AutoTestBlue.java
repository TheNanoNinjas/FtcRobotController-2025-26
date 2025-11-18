//Basically this code is going to be for the red side and it will go forward, strafe, turn, and then run the shoot motors and then it will start operating the gate.
//You will most likely need to tune the gate because I don't have the timing really done for that. Also I coded it so that if the distance sensor sees something within 6 inches it will stop and then back up.
// You can change the distance for the sensor though, as well as the values for the distance. If you are going to change values, change Target Y inches, Strafe to X, and then turnToHeading.-Avi

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="AutoCodeDecodeBlue", group="Linear Opmode")
public class AutoTestBlue extends LinearOpMode {

    private DcMotor fr_motor, fl_motor, br_motor, bl_motor;
    private DcMotor leftShooter, rightShooter;
    private Servo gateServo;
    private GoBildaPinpointDriver odo;
    private Rev2mDistanceSensor distance_Sensor;

    private static final double TARGET_Y_INCHES = 103.0;
    private static final double KP = 0.10;

    @Override
    public void runOpMode() {

        // === Hardware initialization ===
        fr_motor = hardwareMap.get(DcMotor.class, "fr_motor");
        fl_motor = hardwareMap.get(DcMotor.class, "fl_motor");
        br_motor = hardwareMap.get(DcMotor.class, "br_motor");
        bl_motor = hardwareMap.get(DcMotor.class, "bl_motor");
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        distance_Sensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");

        // Odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-82.5, 0.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        // Motor directions
        fr_motor.setDirection(DcMotor.Direction.FORWARD);
        br_motor.setDirection(DcMotor.Direction.REVERSE);
        fl_motor.setDirection(DcMotor.Direction.REVERSE);
        bl_motor.setDirection(DcMotor.Direction.REVERSE);


        gateServo.setDirection(Servo.Direction.REVERSE);
        gateServo.setPosition(0.8);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            //Move forward until target or obstacle
            while (opModeIsActive()) {
                odo.update();
                Pose2D pos = odo.getPosition();

                double currentY = pos.getY(DistanceUnit.INCH);
                double error = TARGET_Y_INCHES - currentY;
                double drivePower = error * KP;
                drivePower = Math.max(-0.3, Math.min(0.3, drivePower));

                double distINCH = distance_Sensor.getDistance(DistanceUnit.INCH);

                boolean targetReached = Math.abs(error) < 1.0;
                boolean obstacleClose = distINCH < 6.0;

                if (targetReached || obstacleClose) {
                    stopMotors();
                    telemetry.addLine("Stopped!");
                    if (targetReached) telemetry.addLine("Reason: Target Reached");
                    if (obstacleClose) {
                        telemetry.addLine("Reason: Obstacle Detected");
                        telemetry.addLine("Backing up...");
                        telemetry.update();

                        // === Back up a little ===
                        driveBackward(0.25);
                        sleep(900); // back up for 0.90 seconds
                        stopMotors();
                    }
                    telemetry.update();
                    break;
                }

                driveForward(drivePower);

                telemetry.addData("Current Y (in)", "%.2f", currentY);
                telemetry.addData("Distance Sensor (in)", "%.2f", distINCH);
                telemetry.addData("Heading (deg)", pos.getHeading(AngleUnit.DEGREES));
                telemetry.update();
            }

            // Strafe
            odo.update();
            // double startX = odo.getPosition().getX(DistanceUnit.INCH);
            //strafeToX(startX + 29.0, 0.3); // strafe right

            turnToHeading(47.0,0.2);

            launchArtifacts(10000);


            //  shootArtifacts(1.0,8000);

            // operateGate(3);

            stopMotors();
        }
    }

    // Moving the robot methods
    private void driveForward(double power) {
        fr_motor.setPower(power);
        fl_motor.setPower(power);
        br_motor.setPower(power);
        bl_motor.setPower(power);
    }


    private void driveBackward(double power) {
        fr_motor.setPower(-power);
        fl_motor.setPower(-power);
        br_motor.setPower(-power);
        bl_motor.setPower(-power);
    }

    private void strafe(double power) {
        fr_motor.setPower(-power);
        fl_motor.setPower(power);
        br_motor.setPower(power);
        bl_motor.setPower(-power);
    }

    private void turnToHeading(double targetHeading, double power) {
        odo.update();
        double currentHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
        double error = targetHeading - currentHeading;

        // Normalize error to range -180 to +180
        error = ((error + 180) % 360) - 180;

        while (opModeIsActive() && Math.abs(error) > 1.0) { // 1° tolerance
            odo.update();
            currentHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
            error = targetHeading - currentHeading;
            error = ((error + 180) % 360) - 180;

            double turnPower = power * Math.signum(error); // positive = clockwise, negative = counterclockwise

            // Set motor powers for turn
            fr_motor.setPower(turnPower);
            br_motor.setPower(turnPower);
            fl_motor.setPower(-turnPower);
            bl_motor.setPower(-turnPower);

            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        stopMotors();
    }


    private void strafeToX(double targetXInches, double basePower) {
        odo.update();
        Pose2D startPos = odo.getPosition();
        double startHeading = startPos.getHeading(AngleUnit.DEGREES);

        double currentX = startPos.getX(DistanceUnit.INCH);
        double error = targetXInches - currentX;
        double direction = Math.signum(error);

        telemetry.addLine("Strafe Started");
        telemetry.update();

        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            currentX = pos.getX(DistanceUnit.INCH);
            double currentHeading = pos.getHeading(AngleUnit.DEGREES);
            error = targetXInches - currentX;

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

            fl_motor.setPower(flPower);
            bl_motor.setPower(blPower);
            fr_motor.setPower(frPower);
            br_motor.setPower(brPower);

            telemetry.addData("Target X (in)", targetXInches);
            telemetry.addData("Current X (in)", currentX);
            telemetry.addData("Remaining Distance (in)", "%.2f", error);
            telemetry.addData("Heading", "%.2f", currentHeading);
            telemetry.addData("Heading Error", "%.2f", headingError);
            telemetry.addData("Correction", "%.2f", correction);
            telemetry.update();

            sleep(20);
        }

        stopMotors();
        telemetry.addLine("=== Strafe Completed ===");
        telemetry.update();
    }

    private void shootArtifacts(double power, long timeMS){
        leftShooter.setPower(power);
        rightShooter.setPower(power);
        sleep(timeMS);
        leftShooter.setPower(0.0);
        rightShooter.setPower(0.0);
    }

    private void operateGate(int cycles) {
        for (int i = 0; i < cycles && opModeIsActive(); i++) {
            gateServo.setPosition(0.9);
            sleep(1200);
            gateServo.setPosition(0.75);
            sleep(1000);
        }
    }

    private void launchArtifacts(long timeMS) {
        leftShooter.setPower(1.0);
        rightShooter.setPower(1.0);
        sleep(300);

        gateServo.setPosition(0.8);
        sleep(800);
        gateServo.setPosition(0.6);

        sleep(1000);

        gateServo.setPosition(0.8);
        sleep(800);
        gateServo.setPosition(0.6);

        sleep(1000);

        gateServo.setPosition(0.8);
        sleep(800);
        gateServo.setPosition(0.6);

        telemetry.addData("GateServo position", gateServo.getPosition());
        telemetry.update();

    }
    private void stopMotors() {
        fr_motor.setPower(0);
        fl_motor.setPower(0);
        br_motor.setPower(0);
        bl_motor.setPower(0);
    }
}