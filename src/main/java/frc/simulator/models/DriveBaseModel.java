package frc.simulator.models;

import org.xerosw.util.SettingsValue;
import org.xerosw.util.SettingsValue.SettingsType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.simulator.engine.SimulationEngine;
import frc.simulator.engine.SimulationModel;

public class DriveBaseModel extends SimulationModel {
    private Drive db_ ;

    public DriveBaseModel(SimulationEngine engine, String name, String inst) {
        super(engine, name, inst) ;
    }

    @Override
    public boolean create(SimulationEngine engine) {
        db_ = RobotContainer.getInstance().drivebase() ;
        if (db_ != null) {
            setCreated();
        }
        return db_ != null ;
    }


    @Override
    public boolean processEvent(String name, SettingsValue value) {
        if (name.equals("pose") && value.getType() == SettingsType.String) {
            try {
                String[] parts = value.getString().split(",") ;
                if (parts.length == 3) {
                    double x = Double.parseDouble(parts[0]) ;
                    double y = Double.parseDouble(parts[1]) ;
                    double theta = Double.parseDouble(parts[2]) ;
                    Pose2d p = new Pose2d(x, y, Rotation2d.fromDegrees(theta)) ;
                    db_.setPose(p) ;
                }
            }
            catch(Exception e) {        
            }            
        }
        return true ;
    }

    @Override
    public void run(double dt) {

    }
}
