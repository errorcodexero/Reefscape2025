package frc.simulator.engine;

import org.xerosw.util.SettingsValue;

public class SimulationAssertEvent extends SimulationEvent {

    @SuppressWarnings("unused")
    private String subsystem_;

    @SuppressWarnings("unused")
    private String name_;

    @SuppressWarnings("unused")
    private String setting_;

    @SuppressWarnings("unused")
    private SettingsValue value_;

    @SuppressWarnings("unused")
    private double tolerance_;

    public SimulationAssertEvent(double t, String subsystem, String name, SettingsValue v, double tol) {
        super(t);

        subsystem_ = subsystem;
        name_ = name;
        value_ = v;
        setting_ = null;

        tolerance_ = tol ;
    }

    public SimulationAssertEvent(double t, String subsystem, String name, String setting, double tol) {
        super(t);
        subsystem_ = subsystem;
        name_ = name;
        value_ = null;
        setting_ = setting;

        tolerance_ = tol ;
    }

    public void run(SimulationEngine engine) {
        // ISimulatedSubsystem sub = engine.getRobot().getSubSystem(subsystem_);
        // if (sub == null) {
        //     MessageLogger logger = engine.getMessageLogger();
        //     logger.startMessage(MessageType.Error);
        //     logger.add("AssertFailed: ");
        //     logger.add("subsystem ", subsystem_);
        //     logger.add(" - does not exist in the robot");
        //     logger.endMessage();
        //     engine.addAssertError();
        // } else {
        //     MessageLogger logger = engine.getMessageLogger();
        //     SettingsValue v = sub.getProperty(name_);
        //     if (v == null) {
        //         logger.startMessage(MessageType.Error);
        //         logger.add("AssertFailed: ");
        //         logger.add("subsystem ", subsystem_);
        //         logger.add(" property ", name_);
        //         logger.add(" - subsystem did not contain the given property");
        //         logger.endMessage();
        //         engine.addAssertError();
        //     } else {
        //         boolean pass = false;

        //         if (value_ == null) {
        //             logger.startMessage(MessageType.Error);
        //             logger.add("AssertFailed: ");
        //             logger.add("subsystem ", subsystem_);
        //             logger.add(" property ", name_);
        //             logger.add(" - the params file did not contain the property").addQuoted(setting_) ;
        //             logger.endMessage();
        //             engine.addAssertError();
        //         }

        //         if (v.isDouble()) {
        //             try {
        //                 pass = Math.abs(v.getDouble() - value_.getDouble()) < tolerance_;
        //             } catch (BadParameterTypeException e) {
        //                 // Should never happen
        //                 pass = false;
        //             }
        //         } else {
        //             pass = v.equals(value_);
        //         }

        //         if (!pass) {
        //             logger.startMessage(MessageType.Error);
        //             logger.add("AssertFailed: ");
        //             logger.add("subsystem ").addQuoted(subsystem_) ;
        //             logger.add(", property ").addQuoted(name_) ;
        //             logger.add(", expected ").addQuoted(value_.toString());
        //             logger.add(", got ").addQuoted(v.toString());
        //             logger.endMessage();
        //             engine.addAssertError();
        //         } else {
        //             logger.startMessage(MessageType.Info);
        //             logger.add("AssertPassed: ");
        //             logger.add("subsystem", subsystem_);
        //             logger.add(", property ").addQuoted(name_) ;
        //             logger.add(", value ").addQuoted(value_.toString());
        //             logger.endMessage();
        //             engine.addAssertPassed();
        //         }
        //     }
        // }
    }

    public String toString() {
        return "SimulationAssertEvent";
    }

    public void setTolerance(double v) {
        tolerance_ = v;
    }
}