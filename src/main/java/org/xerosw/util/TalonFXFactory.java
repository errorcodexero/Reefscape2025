package org.xerosw.util;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonFXFactory {
    private final static int kApplyTries = 5 ;
    private static TalonFXFactory factory_ = new TalonFXFactory() ;

    public static TalonFXFactory getFactory() {
        return factory_ ;
    }

    //
    // Creates a new TalonFX motor controller in brake mode
    //
    public TalonFX createTalonFX(int id, String bus, boolean invert, double limit) {
        return createTalonFX(id, bus, invert, limit, 1.0) ;
    }

    public TalonFX createTalonFX(int id, String bus, boolean invert, double limit, double time)  {
        TalonFX fx = new TalonFX(id, bus) ;

        TalonFXConfiguration config = new TalonFXConfiguration() ;       
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake ;

        if (limit != Double.NaN) {
            config.CurrentLimits.SupplyCurrentLimit = limit ;
            config.CurrentLimits.SupplyCurrentLimitEnable = true ;
            config.CurrentLimits.SupplyCurrentLowerLimit = limit ;
            config.CurrentLimits.SupplyCurrentLowerTime = 1.0 ;
        }

        config.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive ;

        checkError(id, "apply", () -> fx.getConfigurator().apply(config));
        return fx ;
    }       

    public TalonFX createTalonFX(int id, boolean invert, double limit) {
        return createTalonFX(id, "", invert, limit) ;
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