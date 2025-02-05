package org.xerosw.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public class TalonFXFactory {
    private final static int kApplyTries = 5 ;
    private static TalonFXFactory factory_ = new TalonFXFactory() ;

    public static TalonFXFactory getFactory() {
        return factory_ ;
    }

    //
    // Creates a new TalonFX motor controller in brake mode
    //
    public TalonFX createTalonFX(int id, String bus, boolean invert, Current limit) {
        return createTalonFX(id, bus, invert, limit, Seconds.of(1.0)) ;
    }

    public TalonFX createTalonFX(int id, String bus, boolean invert, Current limit, Time time)  {
        TalonFX fx = new TalonFX(id, bus) ;

        TalonFXConfiguration config = new TalonFXConfiguration() ;       
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake ;

        config.CurrentLimits.SupplyCurrentLimit = limit.in(Amps) ;
        config.CurrentLimits.SupplyCurrentLimitEnable = true ;
        config.CurrentLimits.SupplyCurrentLowerLimit = limit.in(Amps) ;
        config.CurrentLimits.SupplyCurrentLowerTime = time.in(Seconds) ;

        config.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive ;

        checkError(id, "apply", () -> fx.getConfigurator().apply(config));
        return fx ;
    }       

    public TalonFX createTalonFX(int id, boolean invert, Current limit) {
        return createTalonFX(id, "", invert, limit, Seconds.of(1.0)) ;
    }     

    public static void checkError(int id, String msg, Supplier<StatusCode> toApply) throws RuntimeException {
        StatusCode code = StatusCode.StatusCodeNotInitialized ;
        int tries = kApplyTries ;
        
        do {
            code = toApply.get() ;
        } while (!code.isOK() && --tries > 0)  ;

        if (!code.isOK()) {
            msg = "canid " + id + ": " + msg + " - code " + code.toString() ;
            throw new RuntimeException(msg) ;
        }
    }    
}