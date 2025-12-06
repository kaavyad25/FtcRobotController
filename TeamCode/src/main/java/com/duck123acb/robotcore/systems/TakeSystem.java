package com.duck123acb.robotcore.systems;

import com.duck123acb.robotcore.Motor;

public class TakeSystem {
    Motor leftMotor, rightMotor;

    public TakeSystem(Motor leftMotor, Motor rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    public void spin(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
