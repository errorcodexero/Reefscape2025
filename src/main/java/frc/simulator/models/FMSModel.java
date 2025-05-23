package frc.simulator.models;

import org.xerosw.util.BadParameterTypeException;
import org.xerosw.util.MessageLogger;
import org.xerosw.util.MessageType;
import org.xerosw.util.SettingsValue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

import frc.simulator.engine.SimulationModel;
import frc.simulator.engine.SimulationEngine;

public class FMSModel extends SimulationModel {
    
    private enum FMSState {
        Initializing,
        Start,
        Test,
        BetweenTestAuto,
        Auto,
        BetweenAutoTeleop,
        Teleop,
        Closing,
        Done,
    } ;

    private FMSState state_ ;
    private double period_start_time_ ;
    private double start_time_  ;
    private double auto_time_ ;
    private double teleop_time_ ;
    private double between_time_ ;
    private double test_time_ ;
    private double closing_time_ ;

    public FMSModel(final SimulationEngine engine, final String model, final String inst) {
        super(engine, model, inst);

        start_time_ = 0.0 ;
        auto_time_ = 0.0 ;
        teleop_time_ = 0.0 ;
        between_time_ = 0.0 ;
        test_time_ = 0.0 ;
        closing_time_ = 0.0 ;        
    }

    @Override
    public boolean create(SimulationEngine engine) {
        state_ = FMSState.Initializing ;

        if (hasProperty("autonomous")) {
            auto_time_ = getDoublePropertyWithDefault("autonomous", null, auto_time_) ;
        }
        
        if (hasProperty("start")) {
            start_time_ = getDoublePropertyWithDefault("start", null, start_time_) ;
        }        
        
        if (hasProperty("between")) {
            between_time_ = getDoublePropertyWithDefault("between", null, between_time_) ;
        }     
        
        if (hasProperty("teleop")) {
            teleop_time_ = getDoublePropertyWithDefault("teleop", null, teleop_time_) ;
        }
        
        if (hasProperty("test")) {
            teleop_time_ = getDoublePropertyWithDefault("test", null, test_time_) ;
        }

        boolean attached = false ;
        
        try {
            if (hasProperty("fms")) {
                attached = getBooleanProperty("fms") ;
            }
        }
        catch(Exception ex) {
        }
        DriverStationSim.setFmsAttached(attached);

        try {
            if (hasProperty("ds")) {
                attached = getBooleanProperty("ds") ;
            }
        }
        catch(Exception ex) {
        }
        DriverStationSim.setDsAttached(attached);

        if (hasProperty("alliance")) {
            AllianceStationID id = AllianceStationID.Red1 ;
            SettingsValue v = getProperty("alliance") ;
            if (v.isString()) {
                try {
                    if (v.getString().equals("red1"))
                        id = AllianceStationID.Red1 ;
                    else if (v.getString().equals("red2"))
                        id = AllianceStationID.Red2 ;
                    else if (v.getString().equals("red3"))
                        id = AllianceStationID.Red3 ;
                    else if (v.getString().equals("blue1"))
                        id = AllianceStationID.Blue1 ;
                    else if (v.getString().equals("blue2"))
                        id = AllianceStationID.Blue2 ;
                    else if (v.getString().equals("blue3"))
                        id = AllianceStationID.Blue3 ;                                                
                } 
                catch(Exception ex) {
                    id = AllianceStationID.Red1 ;
                }
            }
            DriverStationSim.setAllianceStationId(id) ;
        }

        setCreated();
        return true;
    }

    public void run(final double dt) {
        final double elapsed = getRobotTime() - period_start_time_ ;
        switch(state_)
        {
            case Initializing:
                DriverStationSim.setTest(false);
                DriverStationSim.setAutonomous(false);
                enableRobot(false);
                period_start_time_ = getRobotTime() ;
                state_ = FMSState.Start ;
                break ;

            case Start:
                if (elapsed >= start_time_)
                {
                    if (test_time_ > 0.0) {
                        DriverStationSim.setTest(true);
                        DriverStationSim.setAutonomous(false);
                        enableRobot(true);
                        state_ = FMSState.Test ;
                    } else {
                        if (auto_time_ > 0.0) {
                            DriverStationSim.setAutonomous(true);
                            DriverStationSim.setTest(false);
                            enableRobot(true);
                            state_ = FMSState.Auto ;                        
                        }
                        else {
                            DriverStationSim.setAutonomous(false);
                            DriverStationSim.setTest(false);
                            enableRobot(true);
                            state_ = FMSState.Teleop ;                            
                        }
                    }
                    period_start_time_ = getRobotTime() ;
                }
                break ;

            case Test:
                if (elapsed >= test_time_) 
                {
                    DriverStationSim.setTest(false);
                    enableRobot(false);
                    state_ = FMSState.BetweenTestAuto ;
                    period_start_time_ = getRobotTime() ;
                }
                break ;

            case BetweenTestAuto:
                if (elapsed >= between_time_) 
                {
                    if (auto_time_ > 0.0) {
                        DriverStationSim.setAutonomous(true);
                        DriverStationSim.setTest(false);
                        enableRobot(true);
                        state_ = FMSState.Auto ;
                    } else {
                        DriverStationSim.setAutonomous(false);
                        DriverStationSim.setTest(false);
                        enableRobot(true);
                        state_ = FMSState.Teleop ;                             
                    }
                    period_start_time_ = getRobotTime() ;
                }
                break ;

            case Auto:
                if (elapsed >= auto_time_)
                {
                    DriverStationSim.setAutonomous(false);
                    enableRobot(false);
                    state_ = FMSState.BetweenAutoTeleop ;
                    period_start_time_ = getRobotTime() ;
                }            
                break ;

            case BetweenAutoTeleop:
                if (elapsed >= between_time_)
                {
                    DriverStationSim.setAutonomous(false);
                    DriverStationSim.setTest(false);
                    enableRobot(true);
                    state_ = FMSState.Teleop ;
                    period_start_time_ = getRobotTime() ;
                }              
                break ;

            case Teleop:
                if (elapsed >= teleop_time_)
                {
                    DriverStationSim.setAutonomous(false);
                    DriverStationSim.setTest(false);                    
                    enableRobot(false);
                    state_ = FMSState.Closing ;
                    period_start_time_ = getRobotTime() ;
                }               
                break ;

            case Closing:
                if (elapsed >= closing_time_)
                {
                    state_ = FMSState.Done ;
                    period_start_time_ = getRobotTime() ;

                    getEngine().exitSimulator();
                }               
                break ;            

            case Done:
                break ;                                              
        }        
    }

    @Override
    public boolean processEvent(final String name, final SettingsValue value) {
        boolean ret = false;

        if (name.equals("autonomous")) {
            auto_time_ = getDoublePropertyWithDefault(name, value, auto_time_) ;
            ret = true ;
        }
        else if (name.equals("start")) {
            start_time_ = getDoublePropertyWithDefault(name, value, start_time_) ;
            ret = true ;
        }
        else if (name.equals("between")) {
            between_time_ = getDoublePropertyWithDefault(name, value, between_time_) ;
            ret = true ;
        }
        else if (name.equals("teleop")) {
            teleop_time_ = getDoublePropertyWithDefault(name, value, teleop_time_) ;
            ret = true ;
        }
        else if (name.equals("test")) {
            test_time_ = getDoublePropertyWithDefault(name, value, test_time_) ;
            ret = true ;
        }
        else if (name.equals("fms")) {
            if (!value.isBoolean()) {
                final MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a boolean").endMessage();
            }
            else {
                try {
                    DriverStationSim.setFmsAttached(value.getBoolean());
                } catch (final BadParameterTypeException e) {
                }
            }
        }
        else if (name.equals("ds")) {
            if (!value.isBoolean()) {
                final MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a boolean").endMessage();
            }
            else {
                try {
                    DriverStationSim.setDsAttached(value.getBoolean());
                } catch (final BadParameterTypeException e) {
                }
            }
        }        
        else if (name.equals("alliance")) {
            AllianceStationID id = AllianceStationID.Red1 ;
            if (value.isString()) {
                try {
                    if (value.getString().equals("red1"))
                        id = AllianceStationID.Red1 ;
                    else if (value.getString().equals("red2"))
                        id = AllianceStationID.Red2 ;
                    else if (value.getString().equals("red3"))
                        id = AllianceStationID.Red3 ;
                    else if (value.getString().equals("blue1"))
                        id = AllianceStationID.Blue1 ;
                    else if (value.getString().equals("blue2"))
                        id = AllianceStationID.Blue2 ;
                    else if (value.getString().equals("blue3"))
                        id = AllianceStationID.Blue3 ;                                                
                } 
                catch(Exception ex) {
                    id = AllianceStationID.Red1 ;
                }
            }
            DriverStationSim.setAllianceStationId(id) ;
        }

        return ret ;
    }


    private void enableRobot(boolean enable) {
        DriverStationSim.setEnabled(enable);
        if (enable) {
            com.ctre.phoenix6.unmanaged.Unmanaged.feedEnable(100000);
        }
    }
}
