package org.firstinspires.ftc.teamcode.mechanisms;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Intaker {
    private final RobotHardware robot;

    public Intaker(RobotHardware robot) {
        this.robot = robot;
    }

    public void startPushing() {
        robot.intakeMotor.setPower(1);
    }

    public void startPushingAuto(long timeMS) throws InterruptedException {
        robot.intakeMotor.setPower(1.0);
        sleep(timeMS);
    }
public void startIntakeAuto(double power){
        robot.intakeMotor.setPower(power);
}

    public void stopPushing() {
        robot.intakeMotor.setPower(0.0);
    }

    public void reversePush() {
        robot.intakeMotor.setPower(-0.55);
    }

    public void setPushPower(double power) {
        robot.intakeMotor.setPower(power);
    }
}