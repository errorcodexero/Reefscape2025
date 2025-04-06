package frc.simulator.engine;

import java.util.ArrayList;
import java.util.List;

import org.xerosw.util.MessageLogger;
import org.xerosw.util.MessageType;

import edu.wpi.first.hal.simulation.SimulatorJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Robot;

public class SimulationEngine {
    public static final String LoggerName = "simulator" ;
    public static final String NetworkTableName = "XeroSim" ;
    private static SimulationEngine the_one_ = null ;

    private int logger_id_ ;

    private Robot robot_ ;
    private ModelManager models_ ;
    private EventsManager events_ ;

    private List<SimulationModel> active_models_ ;
    private int failed_count_ ;
    private int passed_count_ ;    

    private SimulationEngine(Robot robot) {
        robot_ = robot ;

        MessageLogger logger = MessageLogger.getTheMessageLogger() ;
        logger_id_ = logger.registerSubsystem(LoggerName) ;
        logger.startMessage(MessageType.Info).add("Starting the simulation engine").endMessage();

        logger.enableSubsystem(LoggerName) ;

        models_ = new ModelManager(this);
        events_ = new EventsManager(this);

        active_models_ = new ArrayList<SimulationModel>() ;

        failed_count_ = 0 ;
        passed_count_ = 0 ;
    }

    static public SimulationEngine getInstance() {
        return the_one_;
    }

    static public void initializeSimulator(Robot robot) {
        the_one_ = new SimulationEngine(robot);
    }

    public void addAssertError() {
        failed_count_++ ;
    }

    public void addAssertPassed() {
        passed_count_++ ;
    }

    public void exitSimulator() {
        int code = 0 ;

        if (failed_count_ == 0)
        {
            if (events_.size() > 0) {
                MessageLogger.getTheMessageLogger().startMessage(MessageType.Info).add("Simulation failed").endMessage();
                if (passed_count_ > 0)
                    MessageLogger.getTheMessageLogger().startMessage(MessageType.Info).add("    ").add(passed_count_).add(" asserts passed").endMessage();
                MessageLogger.getTheMessageLogger().startMessage(MessageType.Info).add("    ").add("but there were", events_.size()).add(" events left to be processed").endMessage();
                code = 1 ;
            }
            else {
                MessageLogger.getTheMessageLogger().startMessage(MessageType.Info).add("Simulation completed sucessfully").endMessage();
                MessageLogger.getTheMessageLogger().startMessage(MessageType.Info).add("    ").add(passed_count_).add(" asserts passed").endMessage();
            }
        }
        else
        {
            code = 1 ;
            MessageLogger.getTheMessageLogger().startMessage(MessageType.Info).add("Simulation failed with " + failed_count_ + " errors").endMessage();
            MessageLogger.getTheMessageLogger().startMessage(MessageType.Info).add("    ").add(passed_count_).add(" asserts passed").endMessage();
            MessageLogger.getTheMessageLogger().startMessage(MessageType.Info).add("    ").add(failed_count_).add(" asserts failed").endMessage();

            if (events_.size() > 0) {
                MessageLogger.getTheMessageLogger().startMessage(MessageType.Info).add("    ").add("there were", events_.size()).add(" events left to be processed") ;
            }
        }

        //
        // If the robot code issued any errors during the simulation, return a failed status as well so we track
        // these down
        //
        if (MessageLogger.getTheMessageLogger().getErrorMessageCount() > 0)
            code = 1 ;

        java.lang.System.exit(code) ;
    }

    public ModelFactory getModelFactory() {
        return models_.getFactory() ;
    }

    public void createModels() {
        for(SimulationModel model : active_models_) {
            if (!model.isCreated()) {
                MessageLogger.getTheMessageLogger().startMessage(MessageType.Debug, logger_id_) ;
                MessageLogger.getTheMessageLogger().add("late model creation - model ").addQuoted(model.getModelName()) ;
                MessageLogger.getTheMessageLogger().add(" instance ").addQuoted(model.getInstanceName()).endMessage();   
                try {             
                    model.create(this) ;
                }
                catch(Exception ex) {
                    MessageLogger.getTheMessageLogger().startMessage(MessageType.Error) ;
                    MessageLogger.getTheMessageLogger().add("model ").addQuoted(model.getModelName()) ;
                    MessageLogger.getTheMessageLogger().add(", instance ").addQuoted(model.getInstanceName()) ;
                    MessageLogger.getTheMessageLogger().add(" - failed creation ").addQuoted(ex.getMessage()) ;
                    MessageLogger.getTheMessageLogger().endMessage();
                    MessageLogger.getTheMessageLogger().logStackTrace(ex.getStackTrace());
                }
            }
        }
    }

    public void addModel(SimulationModel model) {
        if (model.hasProperty("create_early")) {
            MessageLogger.getTheMessageLogger().startMessage(MessageType.Debug, logger_id_) ;
            MessageLogger.getTheMessageLogger().add("create model early call, model ").addQuoted(model.getModelName()) ;
            MessageLogger.getTheMessageLogger().add(" instance ").addQuoted(model.getInstanceName()).endMessage();
            try {
                model.create(this) ;
            }
            catch(Exception ex) {
                MessageLogger.getTheMessageLogger().startMessage(MessageType.Error, logger_id_) ;
                MessageLogger.getTheMessageLogger().add("exception thrown creating model ").addQuoted(model.getModelName()) ;
                MessageLogger.getTheMessageLogger().add(" instance ").addQuoted(model.getInstanceName()).add(" - " + ex.getMessage()).endMessage();
                MessageLogger.getTheMessageLogger().logStackTrace(ex.getStackTrace());
            }

        }
        active_models_.add(model) ;
        DriverStationSim.notifyNewData();
    }

    //
    // This is the amount of time to run forward to have the simulation
    // models catch up with the simulation
    //
    public void run(double dt) {
        SimulatorJNI.pauseTiming();

        for(SimulationModel model : active_models_) {
            model.startCycle();
        }

        processEvents() ;
        runModels(dt) ;
        DriverStationSim.notifyNewData() ;

        for(SimulationModel model : active_models_)
            model.endCycle();        

        SimulatorJNI.resumeTiming();
    }

    public MessageLogger getMessageLogger() {
        return MessageLogger.getTheMessageLogger() ;
    }

    public Robot getRobot() {
        return robot_ ;
    }

    public double getSimulationTime() {
        return Timer.getFPGATimestamp() ;
    }

    public SimulationModel getModelByNameInst(String model, String inst) {
        for(SimulationModel m: active_models_) {
            if (m.getModelName().equals(model) && m.getInstanceName().equals(inst))
                return m ;
        }

        return null ;
    }

    private void processEvents() {
        while (events_.size() > 0) {
            SimulationEvent ev = events_.getFirstEvent() ;
            if (ev.getTime() > getSimulationTime())
                break ;

            MessageLogger.getTheMessageLogger().startMessage(MessageType.Debug, logger_id_) ;
            MessageLogger.getTheMessageLogger().add("processing event ").addQuoted(ev.toString()) ;
            MessageLogger.getTheMessageLogger().endMessage();
            ev.run(this) ;
            events_.removeFirstEvent();
        }
    }

    private void runModels(double dt) {
        for(SimulationModel m : active_models_) {
            if (m.isCreated())
                m.run(dt) ;
            else {
                if (!m.warnedNotRun()) {
                    MessageLogger.getTheMessageLogger().startMessage(MessageType.Error) ;
                    MessageLogger.getTheMessageLogger().add("did not run model ").addQuoted(m.getModelName()) ;
                    MessageLogger.getTheMessageLogger().add(" instance ").addQuoted(m.getInstanceName()) ;
                    MessageLogger.getTheMessageLogger().add(" - model not created").endMessage(); 
                    m.setWarnedNotRun(); 
                }
            }
        }
    }    

    private void readModelFile(String file) {
        models_.readModelFile(file) ;
    }

    private void readEventsFile(String file) {
        events_.readEventsFile(file) ;
    }
    
    public void initAll(String simfile) {
        readModelFile("src/sim/robot.json") ;
        readEventsFile("src/sim/sims/" + simfile + ".json") ;
    }
}
