package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Intaker {
    private RobotHardware robot;

    public Intaker(RobotHardware robot) {
        this.robot = robot;
    }

    public void startPushing() {
        robot.intakeMotor.setPower(1.0);
    }

    public void stopPushing() {
        robot.intakeMotor.setPower(0.0);
    }

    public void reversePush() {
        robot.intakeMotor.setPower(-.5);
    }

    public void setPushPower(double power) {
        robot.intakeMotor.setPower(power);
    }
}