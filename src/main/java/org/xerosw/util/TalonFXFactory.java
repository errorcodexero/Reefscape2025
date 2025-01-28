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

    private int motor_created_ = 0 ;

    public static TalonFXFactory getFactory() {
        return factory_ ;
    }

    //
    // Creates a new TalonFX motor controller in brake mode
    //
    public TalonFX createTalonFX(int id, String bus, boolean invert, double limit) throws Exception {
        return createTalonFX(id, bus, invert, limit, 1.0) ;
    }

    public TalonFX createTalonFX(int id, String bus, boolean invert, double limit, double time) throws Exception {
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

        checkError(id, "TalonFXMotorController - apply configuration", () -> fx.getConfigurator().apply(config), -1);        

        motor_created_++ ;
        return fx ;
    }       

    public TalonFX createTalonFX(int id, String bus, boolean invert) throws Exception {
        return createTalonFX(id, invert, Double.NaN) ;
    }     

    public TalonFX createTalonFX(int id, boolean invert) throws Exception {
        return createTalonFX(id, "", invert, Double.NaN) ;
    }   

    public TalonFX createTalonFX(int id, boolean invert, double limit) throws Exception {
        return createTalonFX(id, "", invert, limit) ;
    }     

    private void checkError(int id, String msg, Supplier<StatusCode> toApply, int reps) throws Exception {
        StatusCode code = StatusCode.StatusCodeNotInitialized ;
        int tries = (reps == -1 ? kApplyTries : reps) ;
        do {
            code = toApply.get() ;
        } while (!code.isOK() && --tries > 0)  ;

        if (!code.isOK()) {
            msg = msg + " - code " + code.toString()  + " - count " + motor_created_ + " - id " + id ;
            throw new Exception(msg) ;
        }
    }    
}