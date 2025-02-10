package org.xerosw.hid.mapping;

import org.xerosw.hid.IMappedGamepad;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PlaystationMappedGamepad extends CommandPS5Controller implements IMappedGamepad {

    public PlaystationMappedGamepad(int port) {
        super(port);
    }

    @Override
    public Trigger a() {
        return cross();
    }

    @Override
    public Trigger b() {
        return circle();
    }

    @Override
    public Trigger x() {
        return square();
    }

    @Override
    public Trigger y() {
        return triangle();
    }

    @Override
    public Trigger leftBumper() {
        return L1();
    }

    @Override
    public Trigger rightBumper() {
        return R1();
    }

    @Override
    public Trigger back() {
        return create();
    }

    @Override
    public Trigger start() {
        return options();
    }

    @Override
    public Trigger leftStick() {
        return L3();
    }

    @Override
    public Trigger rightStick() {
        return R3();
    }

    @Override
    public Trigger leftTrigger(double threshold) {
        return new Trigger(() -> getL2Axis() > threshold);
    }

    @Override
    public Trigger leftTrigger() {
        return L2();
    }

    @Override
    public Trigger rightTrigger(double threshold) {
        return new Trigger(() -> getR2Axis() > threshold);
    }

    @Override
    public Trigger rightTrigger() {
        return R2();
    }

    @Override
    public double getLeftTriggerAxis() {
        return getL2Axis();
    }

    @Override
    public double getRightTriggerAxis() {
        return getR2Axis();
    }
    
}
