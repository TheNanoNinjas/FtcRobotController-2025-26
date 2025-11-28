package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "NanoNinjasMainCode")
@Disabled
public class FTCNanoNinjasCode extends LinearOpMode {

    private DcMotor intakeMotor;
 //   private Servo angleServo;
    private DcMotor leftShooter;
    private DcMotor rightShooter;
    private DcMotor fl_motor;
    private DcMotor fr_motor;
    private DcMotor bl_motor;
    private DcMotor br_motor;
    private DcMotor wheelMotor;
//    private double angleServoPos = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        waitForStart();

        while (opModeIsActive()) {
            moveRobot();
         //   pushArtifacts();
            pushArtifactsSecond();
            intakeArtifacts();
            launchArtifactsFar();
            launchArtifacts();
            manualDrop();
            shootArtifacts();

           // controlAngleServo();
         //   telemetry.addData("AngleServo Position", angleServo.getPosition());
           // telemetry.update();
        }
    }

    private void initializeHardware() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");

        wheelMotor = hardwareMap.get(DcMotor.class, "wheelMotor");
        fl_motor = hardwareMap.get(DcMotor.class, "fl_motor");
        fr_motor = hardwareMap.get(DcMotor.class, "fr_motor");
        bl_motor = hardwareMap.get(DcMotor.class, "bl_motor");
        br_motor = hardwareMap.get(DcMotor.class, "br_motor");

      //  angleServo = hardwareMap.get(Servo.class, "angleServo");
        //angleServo.setPosition(0.75);

        fl_motor.setDirection(DcMotor.Direction.REVERSE);
        fr_motor.setDirection(DcMotor.Direction.FORWARD);
        bl_motor.setDirection(DcMotor.Direction.FORWARD);
        br_motor.setDirection(DcMotor.Direction.REVERSE);



      /*  fl_motor.setDirection(DcMotor.Direction.FORWARD);
        fr_motor.setDirection(DcMotor.Direction.REVERSE);
        bl_motor.setDirection(DcMotor.Direction.REVERSE);
        br_motor.setDirection(DcMotor.Direction.FORWARD);*/

        rightShooter.setDirection(DcMotor.Direction.FORWARD);
        leftShooter.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Hardware", "Initialized");
        telemetry.update();
    }

  /*  private void controlAngleServo() {
        double stick = -gamepad2.left_stick_y;
        double deadzone = 0.05;

        if (Math.abs(stick) < deadzone) stick = 0;

        double speed = 0.005;
        angleServoPos += stick * speed;
        angleServoPos = Math.max(0.0, Math.min(1.0, angleServoPos));

        angleServo.setPosition(angleServoPos);
        telemetry.addData("AngleServo Position", angleServoPos);
    }
*/

    private void moveRobot() {
        double drive = -gamepad1.left_stick_x;
        double strafe = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double powerScale = 0.45;

        double leftFrontPower = (drive + strafe + turn) * powerScale;
        double rightFrontPower = (drive - strafe - turn) * powerScale;
        double leftRearPower = (drive - strafe + turn) * powerScale;
        double rightRearPower = (drive + strafe - turn) * powerScale;

        double max = Math.max(1.0, Math.max(
                Math.abs(leftFrontPower),
                Math.max(Math.abs(rightFrontPower),
                        Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)))));

        leftFrontPower /= max;
        rightFrontPower /= max;
        leftRearPower /= max;
        rightRearPower /= max;

        fl_motor.setPower(leftFrontPower);
        fr_motor.setPower(rightFrontPower);
        bl_motor.setPower(leftRearPower);
        br_motor.setPower(rightRearPower);
    }

      /*  private void strafeRobo(){
            double stick = -gamepad1.left_stick_x;
                fl_motor.setPower(0.45);
                fr_motor.setPower(0.45);
                bl_motor.setPower(0.45);
                br_motor.setPower(0.45);

                fl_motor.setDirection(DcMotor.Direction.FORWARD);
                fr_motor.setDirection(DcMotor.Direction.FORWARD);
                bl_motor.setDirection(DcMotor.Direction.FORWARD);
                br_motor.setDirection(DcMotor.Direction.REVERSE);


            fl_motor.setDirection(DcMotor.Direction.REVERSE);
            fr_motor.setDirection(DcMotor.Direction.FORWARD);
            bl_motor.setDirection(DcMotor.Direction.FORWARD);
            br_motor.setDirection(DcMotor.Direction.REVERSE);
         }*/
    private void shootArtifacts() {
        if (gamepad2.right_bumper) {
            leftShooter.setPower(0.7);
            rightShooter.setPower(0.7);
        } else {
            leftShooter.setPower(0.0);
            rightShooter.setPower(0.0);
        }
    }

    private void intakeArtifacts() {
        if (gamepad1.left_bumper) {
            intakeMotor.setPower(1.0);

        } else {
            intakeMotor.setPower(0.0);

        }
    }

 /*   private void pushArtifacts() {
        if (gamepad2.cross) {
            pushServoFar.setPower(1.0);
        } else {
            pushServoFar.setPower(0.0);
        }
    }
*/
    private void pushArtifactsSecond() {
        if (gamepad1.triangle) {
            wheelMotor.setPower(1.0);
        } else {
            wheelMotor.setPower(0.0);
        }
    }

    private void launchArtifacts() {
        if (gamepad2.square) {
            rightShooter.setPower(0.6);
            leftShooter.setPower(0.6);
        } else {
            rightShooter.setPower(0);
            leftShooter.setPower(0);
        }
    }

    private void manualDrop() {
        if (gamepad2.right_stick_button) {
            intakeMotor.setPower(-1.0);
            wheelMotor.setPower(-1.0);
            sleep(200);
        } else {
            intakeMotor.setPower(0.0);
            wheelMotor.setPower(0.0);

        }
    }

    private void launchArtifactsFar() {
        if (gamepad2.circle) {
            rightShooter.setPower(0.78);
            leftShooter.setPower(0.78);
            intakeMotor.setPower(1.0);
            wheelMotor.setPower(1.0);
        } else {
            rightShooter.setPower(0);
            leftShooter.setPower(0);
            intakeMotor.setPower(0);
            wheelMotor.setPower(0.0);
        }
    }
}
