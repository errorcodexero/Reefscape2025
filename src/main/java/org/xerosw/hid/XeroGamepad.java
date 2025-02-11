package org.xerosw.hid;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XeroGamepad {
    
    private final CommandXboxController gamepad_;
    private final int port_;

    private boolean locked_;
    
    public XeroGamepad(int port) {
        port_ = port;
        gamepad_ = new CommandXboxController(port);
        locked_ = false;

        Logger.recordOutput("Gamepad/" + port_ + "/Locked", locked_);
    }

    /**
    * Sets if the inputs from this gamepad is locked from drivers.
    * @param enabled
    */
    public void setLocked(boolean locked) {
        locked_ = locked;
        Logger.recordOutput("Gamepad/" + port_ + "/Locked", locked_);
    }

    /**
    * A command for {@link #setLocked(boolean)}
    * @param locked
    * @return A command to set the lock state of this gamepad.
    */
    public Command setLockCommand(boolean locked) {
        return Commands.runOnce(() -> setLocked(locked));
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

    /*
    * Buttons
    */

    public Trigger a() {
        return ifUnlocked(gamepad_.a());
    }

    public Trigger b() {
        return ifUnlocked(gamepad_.b());
    }

    public Trigger x() {
        return ifUnlocked(gamepad_.x());
    }

    public Trigger y() {
        return ifUnlocked(gamepad_.y());
    }

    public Trigger leftBumper() {
        return ifUnlocked(gamepad_.leftBumper());
    }

    public Trigger rightBumper() {
        return ifUnlocked(gamepad_.rightBumper());
    }

    public Trigger back() {
        return ifUnlocked(gamepad_.back());
    }

    public Trigger start() {
        return ifUnlocked(gamepad_.start());
    }
    
    public Trigger leftStick() {
        return ifUnlocked(gamepad_.leftStick());
    }

    public Trigger rightStick() {
        return ifUnlocked(gamepad_.rightStick());
    }
    
    public Trigger leftTrigger(double threshold) {
        return ifUnlocked(gamepad_.leftTrigger(threshold));
    }

    public Trigger leftTrigger() {
        return ifUnlocked(gamepad_.leftTrigger());
    }

    public Trigger rightTrigger(double threshold) {
        return ifUnlocked(gamepad_.rightTrigger(threshold));
    }

    public Trigger rightTrigger() {
        return ifUnlocked(gamepad_.rightTrigger());
    }

    public Trigger pov(int angle) {
        return ifUnlocked(gamepad_.pov(angle));
    }

    public Trigger povUp() {
        return ifUnlocked(gamepad_.povUp());
    }

    public Trigger povUpRight() {
        return ifUnlocked(gamepad_.povUpRight());
    }

    public Trigger povRight() {
        return ifUnlocked(gamepad_.povRight());
    }

    public Trigger povDownRight() {
        return ifUnlocked(gamepad_.povDownRight());
    }

    public Trigger povDown() {
        return ifUnlocked(gamepad_.povDown());
    }

    public Trigger povDownLeft() {
        return ifUnlocked(gamepad_.povDownLeft());
    }

    public Trigger povLeft() {
        return ifUnlocked(gamepad_.povLeft());
    }

    public Trigger povUpLeft() {
        return ifUnlocked(gamepad_.povUpLeft());
    }

    public Trigger povCenter() {
        return ifUnlocked(gamepad_.povCenter());
    }
    
    /*
    * Axes
    */

    public double getLeftX() {
        return zeroIfLocked(gamepad_.getLeftX());
    }
    
    public double getRightX() {
        return zeroIfLocked(gamepad_.getRightX());
    }
    
    public double getLeftY() {
        return zeroIfLocked(gamepad_.getLeftY());
    }

    public double getRightY() {
        return zeroIfLocked(gamepad_.getRightY());
    }

    public double getLeftTriggerAxis() {
        return zeroIfLocked(gamepad_.getLeftTriggerAxis());
    }

    public double getRightTriggerAxis() {
        return zeroIfLocked(gamepad_.getRightTriggerAxis());
    }

    public void setRumble(RumbleType type, double value) {
        gamepad_.setRumble(type, value);
    }
    

    public boolean isConnected() {
        return gamepad_.isConnected();
    }

    // Utils

    private Trigger ifUnlocked(Trigger trigger) {
        return trigger.and(this::isUnlocked);
    }
    
    private double zeroIfLocked(double axis) {
        return isUnlocked() ? axis : 0.0;
    }
    
}
