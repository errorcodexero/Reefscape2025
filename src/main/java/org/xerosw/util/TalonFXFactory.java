package org.xerosw.util;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

/**
 * This is a class that creates TalonFX motors in different modes, with error checking for configuration calls.
 */
public class TalonFXFactory {

    private static final int kApplyTries = 5;

    /**
     * Creates a new TalonFX motor controller in brake mode.
     * @param id The CAN id of the motor.
     * @param bus The CAN bus the motor is on.
     * @param invert If true, invert the motor.
     * @param currentLimit The supply current limit.
     * @param lowerTime The time limit for the supply current limit.
     * @return The created TalonFX motor controller with the applied configurations.
     * @throws Exception Throws an exception if the motor failed to be configured more than a few times.
     */
    public static TalonFX createTalonFX(int id, String bus, boolean invert, Current currentLimit, Time lowerTime) throws Exception {
        TalonFX fx = new TalonFX(id, bus);
        
        TalonFXConfiguration config = new TalonFXConfiguration();       
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.SupplyCurrentLimit = currentLimit.in(Amps);
        config.CurrentLimits.SupplyCurrentLowerTime = lowerTime.in(Seconds);
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        checkError(id, "TalonFXMotorController - apply configuration", () -> fx.getConfigurator().apply(config), -1);        
        
        return fx;
    }
    
    /**
     * Creates a new TalonFX motor controller in brake mode.
     * @param id The CAN id of the motor.
     * @param bus The CAN bus the motor is on.
     * @param currentLimit The supply current limit.
     * @param lowerTime The time limit for the supply current limit.
     * @return The created TalonFX motor controller with the applied configurations.
     * @throws Exception Throws an exception if the motor failed to be configured more than a few times.
     */
    public static TalonFX createTalonFX(int id, String bus, Current currentLimit, Time lowerTime) throws Exception {
        return createTalonFX(id, bus, false, currentLimit, lowerTime);
    }

    /**
     * Creates a new TalonFX motor controller in brake mode on the default RIO bus.
     * @param id The CAN id of the motor.
     * @return The created TalonFX motor controller with the applied configurations.
     * @throws Exception Throws an exception if the motor failed to be configured more than a few times.
     */
    public static TalonFX createTalonFX(int id, Current currentLimit, Time lowerTime) throws Exception {
        return createTalonFX(id, "", currentLimit, lowerTime);
    }

    /**
     * Creates a new TalonFX motor controller in brake mode on the default RIO bus.
     * @param id The CAN id of the motor.
     * @param invert If true, invert the motor
     * @param currentLimit The supply current limit.
     * @param lowerTime The time limit for the supply current limit.
     * @return The created TalonFX motor controller with the applied configurations.
     * @throws Exception Throws an exception if the motor failed to be configured more than a few times.
     */
    public static TalonFX createTalonFX(int id, boolean invert, Current currentLimit, Time lowerTime) throws Exception {
        return createTalonFX(id, "", invert, currentLimit, lowerTime);
    }

    /**
     * Retries a configuration / checks a status code a specified number of times until it succeeds.
     * @param id The CAN id of the motor.
     * @param msg The message to log on a failure.
     * @param toApply A supplier of the StatusCode you would like to check.
     * @param reps The amount of times it can fail before giving up.
     * @throws Exception Throws an exception if it never succeeds.
     */
    public static void checkError(int id, String msg, Supplier<StatusCode> toApply, int reps) throws Exception {
        StatusCode code = StatusCode.StatusCodeNotInitialized;
        int tries = (reps == -1 ? kApplyTries : reps);
        do {
            code = toApply.get();
        } while (!code.isOK() && --tries > 0);

        if (!code.isOK()) {
            msg = msg + " - code " + code.toString()  + " - id " + id;
            throw new Exception(msg);
        }
    }

    /**
     * Retries a configuration / checks a status code the default number of times until it succeeds.
     * @param id The CAN id of the motor.
     * @param msg The message to log on a failure.
     * @param toApply A supplier of the StatusCode you would like to check.
     * @throws Exception Throws an exception if it never succeeds.
     */
    public static void checkError(int id, String msg, Supplier<StatusCode> toApply) throws Exception {
        checkError(id, msg, toApply, -1);
    }

}