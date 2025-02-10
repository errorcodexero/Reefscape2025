package org.xerosw.hid;

import org.xerosw.hid.mapping.PlaystationMappedGamepad;
import org.xerosw.hid.mapping.XboxMappedGamepad;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XeroGamepad implements IMappedGamepad {
    
    public static enum GamepadType {
        XBOX,
        PLAYSTATION
    }
    
    private final IMappedGamepad gamepad_;
    
    private boolean locked_;
    
    public XeroGamepad(int port) {
        
        locked_ = false;
        
        gamepad_ = switch(guessGamepadType(port)) {
            case PLAYSTATION -> new PlaystationMappedGamepad(port);
            default -> new XboxMappedGamepad(port);
        };
        
    }
    
    private GamepadType guessGamepadType(int port) {
        return DriverStation.getJoystickName(port).equals("Wireless Controller") ? GamepadType.PLAYSTATION : GamepadType.XBOX;
    }
    
    /**
    * Sets if the inputs from this gamepad is locked from drivers.
    * @param enabled
    */
    public void lock(boolean locked) {
        locked_ = locked;
    }
    
    /**
    * A command for {@link #setLocked(boolean)}
    * @param locked
    * @return A command to set the lock state of this gamepad.
    */
    public Command lockCommand(boolean locked) {
        return Commands.runOnce(() -> lock(locked));
    }
    
    /**
    * Whether or not this gamepad is locked from driver input.
    * @return If its locked
    */
    public boolean isLocked() {
        return locked_;
    }
    
    /**
    * Whether or not this gamepad is unlocked from driver input.
    * @return If its unlocked
    */
    public boolean isUnlocked() {
        return !locked_;
    }
    
    private Trigger ifUnlocked(Trigger trigger) {
        return trigger.and(this::isUnlocked);
    }
    
    private double zeroIfLocked(double axis) {
        return isUnlocked() ? axis : 0.0;
    }
    
    /*
    * Buttons
    */
    
    public Trigger a() {
        return ifUnlocked(gamepad_.a());
    }
    
    @Override
    public Trigger b() {
        return ifUnlocked(gamepad_.b());
    }
    
    @Override
    public Trigger x() {
        return ifUnlocked(gamepad_.x());
    }
    
    @Override
    public Trigger y() {
        return ifUnlocked(gamepad_.y());
    }
    
    @Override
    public Trigger leftBumper() {
        return ifUnlocked(gamepad_.leftBumper());
    }
    
    @Override
    public Trigger rightBumper() {
        return ifUnlocked(gamepad_.rightBumper());
    }
    
    @Override
    public Trigger back() {
        return ifUnlocked(gamepad_.back());
    }
    
    @Override
    public Trigger start() {
        return ifUnlocked(gamepad_.start());
    }
    
    @Override
    public Trigger leftStick() {
        return ifUnlocked(gamepad_.leftStick());
    }
    
    @Override
    public Trigger rightStick() {
        return ifUnlocked(gamepad_.rightStick());
    }
    
    @Override
    public Trigger leftTrigger(double threshold) {
        return ifUnlocked(gamepad_.leftTrigger(threshold));
    }
    
    @Override
    public Trigger leftTrigger() {
        return ifUnlocked(gamepad_.leftTrigger());
    }
    
    @Override
    public Trigger rightTrigger(double threshold) {
        return ifUnlocked(gamepad_.rightTrigger(threshold));
    }
    
    @Override
    public Trigger rightTrigger() {
        return ifUnlocked(gamepad_.rightTrigger());
    }
    
    @Override
    public Trigger pov(int angle) {
        return ifUnlocked(gamepad_.pov(angle));
    }
    
    @Override
    public Trigger povUp() {
        return ifUnlocked(gamepad_.povUp());
    }
    
    @Override
    public Trigger povUpRight() {
        return ifUnlocked(gamepad_.povUpRight());
    }
    
    @Override
    public Trigger povRight() {
        return ifUnlocked(gamepad_.povRight());
    }
    
    @Override
    public Trigger povDownRight() {
        return ifUnlocked(gamepad_.povDownRight());
    }
    
    @Override
    public Trigger povDown() {
        return ifUnlocked(gamepad_.povDown());
    }
    
    @Override
    public Trigger povDownLeft() {
        return ifUnlocked(gamepad_.povDownLeft());
    }
    
    @Override
    public Trigger povLeft() {
        return ifUnlocked(gamepad_.povLeft());
    }
    
    @Override
    public Trigger povUpLeft() {
        return ifUnlocked(gamepad_.povUpLeft());
    }
    
    @Override
    public Trigger povCenter() {
        return ifUnlocked(gamepad_.povCenter());
    }
    
    /*
    * Axes
    */
    
    @Override
    public double getLeftX() {
        return zeroIfLocked(gamepad_.getLeftX());
    }
    
    @Override
    public double getRightX() {
        return zeroIfLocked(gamepad_.getRightX());
    }
    
    @Override
    public double getLeftY() {
        return zeroIfLocked(gamepad_.getLeftY());
    }
    
    @Override
    public double getRightY() {
        return zeroIfLocked(gamepad_.getRightY());
    }
    
    @Override
    public double getLeftTriggerAxis() {
        return zeroIfLocked(gamepad_.getLeftTriggerAxis());
    }
    
    @Override
    public double getRightTriggerAxis() {
        return zeroIfLocked(gamepad_.getRightTriggerAxis());
    }
    
    @Override
    public void setRumble(RumbleType type, double value) {
        gamepad_.setRumble(type, value);
    }
    
    @Override
    public boolean isConnected() {
        return gamepad_.isConnected();
    }
    
}
