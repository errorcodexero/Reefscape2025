package org.xerosw.hid;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IMappedGamepad {
    
    /**
    * Constructs a Trigger instance around the A button's digital signal.
    *
    * @return a Trigger instance representing the A button's digital signal attached
    *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
    */
    public Trigger a();
    
    /**
    * Constructs a Trigger instance around the B button's digital signal.
    *
    * @return a Trigger instance representing the B button's digital signal attached
    *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
    */
    public Trigger b();
    
    /**
    * Constructs a Trigger instance around the X button's digital signal.
    *
    * @return a Trigger instance representing the X button's digital signal attached
    *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
    */
    public Trigger x();
    
    /**
    * Constructs a Trigger instance around the Y button's digital signal.
    *
    * @return a Trigger instance representing the Y button's digital signal attached
    *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
    */
    public Trigger y();
    
    /**
    * Constructs a Trigger instance around the left bumper button's digital signal.
    *
    * @return a Trigger instance representing the left bumper button's digital signal attached
    *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
    */
    public Trigger leftBumper();
    
    /**
    * Constructs a Trigger instance around the right bumper button's digital signal.
    *
    * @return a Trigger instance representing the right bumper button's digital signal attached
    *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
    */
    public Trigger rightBumper();
    
    /**
    * Constructs a Trigger instance around the back button's digital signal.
    *
    * @return a Trigger instance representing the back button's digital signal attached
    *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
    */
    public Trigger back();
    
    /**
    * Constructs a Trigger instance around the start button's digital signal.
    *
    * @return a Trigger instance representing the start button's digital signal attached
    *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
    */
    public Trigger start();
    
    /**
    * Constructs a Trigger instance around the left stick button's digital signal.
    *
    * @return a Trigger instance representing the left stick button's digital signal attached
    *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
    */
    public Trigger leftStick();
    
    /**
    * Constructs a Trigger instance around the right stick button's digital signal.
    *
    * @return a Trigger instance representing the right stick button's digital signal attached
    *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
    */
    public Trigger rightStick();
    
    /**
    * Constructs a Trigger instance around the axis value of the left trigger. The returned
    * trigger will be true when the axis value is greater than {@code threshold}.
    *
    * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
    *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
    * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
    *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
    *     button loop}.
    */
    public Trigger leftTrigger(double threshold);
    
    /**
    * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
    * will be true when the axis value is greater than 0.5.
    *
    * @return a Trigger instance that is true when the left trigger's axis exceeds 0.5, attached to
    *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
    */
    public Trigger leftTrigger();
    
    /**
    * Constructs a Trigger instance around the axis value of the right trigger. The returned
    * trigger will be true when the axis value is greater than {@code threshold}.
    *
    * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
    *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
    * @return a Trigger instance that is true when the right trigger's axis exceeds the provided
    *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
    *     button loop}.
    */
    public Trigger rightTrigger(double threshold);
    
    /**
    * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
    * will be true when the axis value is greater than 0.5.
    *
    * @return a Trigger instance that is true when the right trigger's axis exceeds 0.5, attached to
    *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
    */
    public Trigger rightTrigger();
    
    /**
    * Get the X axis value of left side of the controller.
    *
    * @return The axis value.
    */
    public double getLeftX();
    
    /**
    * Get the X axis value of right side of the controller.
    *
    * @return The axis value.
    */
    public double getRightX();
    
    /**
    * Get the Y axis value of left side of the controller.
    *
    * @return The axis value.
    */
    public double getLeftY();
    
    /**
    * Get the Y axis value of right side of the controller.
    *
    * @return The axis value.
    */
    public double getRightY();
    
    /**
    * Get the left trigger axis value of the controller. Note that this axis is bound to the
    * range of [0, 1] as opposed to the usual [-1, 1].
    *
    * @return The axis value.
    */
    public double getLeftTriggerAxis();
    
    /**
    * Get the right trigger axis value of the controller. Note that this axis is bound to the
    * range of [0, 1] as opposed to the usual [-1, 1].
    *
    * @return The axis value.
    */
    public double getRightTriggerAxis();
    
    /**
    * Constructs a Trigger instance based around this angle of the default (index 0) POV on the HID,
    * attached to {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
    * loop}.
    *
    * <p>The POV angles start at 0 in the up direction, and increase clockwise (e.g. right is 90,
    * upper-left is 315).
    *
    * @param angle POV angle in degrees, or -1 for the center / not pressed.
    * @return a Trigger instance based around this angle of a POV on the HID.
    */
    public Trigger pov(int angle);
    
    /**
    * Constructs a Trigger instance based around the 0 degree angle (up) of the default (index 0) POV
    * on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default command
    * scheduler button loop}.
    *
    * @return a Trigger instance based around the 0 degree angle of a POV on the HID.
    */
    public Trigger povUp();
    
    /**
    * Constructs a Trigger instance based around the 45 degree angle (right up) of the default (index
    * 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default
    * command scheduler button loop}.
    *
    * @return a Trigger instance based around the 45 degree angle of a POV on the HID.
    */
    public Trigger povUpRight();
    
    /**
    * Constructs a Trigger instance based around the 90 degree angle (right) of the default (index 0)
    * POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default command
    * scheduler button loop}.
    *
    * @return a Trigger instance based around the 90 degree angle of a POV on the HID.
    */
    public Trigger povRight();
    
    /**
    * Constructs a Trigger instance based around the 135 degree angle (right down) of the default
    * (index 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the
    * default command scheduler button loop}.
    *
    * @return a Trigger instance based around the 135 degree angle of a POV on the HID.
    */
    public Trigger povDownRight();
    
    /**
    * Constructs a Trigger instance based around the 180 degree angle (down) of the default (index 0)
    * POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default command
    * scheduler button loop}.
    *
    * @return a Trigger instance based around the 180 degree angle of a POV on the HID.
    */
    public Trigger povDown();
    
    /**
    * Constructs a Trigger instance based around the 225 degree angle (down left) of the default
    * (index 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the
    * default command scheduler button loop}.
    *
    * @return a Trigger instance based around the 225 degree angle of a POV on the HID.
    */
    public Trigger povDownLeft();
    
    /**
    * Constructs a Trigger instance based around the 270 degree angle (left) of the default (index 0)
    * POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default command
    * scheduler button loop}.
    *
    * @return a Trigger instance based around the 270 degree angle of a POV on the HID.
    */
    public Trigger povLeft();

    /**
    * Constructs a Trigger instance based around the 315 degree angle (left up) of the default (index
    * 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default
    * command scheduler button loop}.
    *
    * @return a Trigger instance based around the 315 degree angle of a POV on the HID.
    */
    public Trigger povUpLeft();
    
    /**
    * Constructs a Trigger instance based around the center (not pressed) position of the default
    * (index 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the
    * default command scheduler button loop}.
    *
    * @return a Trigger instance based around the center position of a POV on the HID.
    */
    public Trigger povCenter();
    
    /**
    * Set the rumble output for the HID. The DS currently supports 2 rumble values, left rumble and
    * right rumble.
    *
    * @param type Which rumble value to set
    * @param value The normalized value (0 to 1) to set the rumble to
    */
    public void setRumble(GenericHID.RumbleType type, double value);
    
    /**
    * Get if the HID is connected.
    *
    * @return true if the HID is connected
    */
    public boolean isConnected();
}
