package frc.robot.commands.tests;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.GoToCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class ManualPlaceReadyCmd extends Command {
    private int level_ ;
    private ManipulatorSubsystem m_ ;
    private GoToCmd cmd_ ;

    private static final Distance L4Dist = Centimeters.of(194.0) ;
    private static final Angle L4Angle = Degrees.of(60.0) ;

    private static final Distance L3Dist = Centimeters.of(130.0) ;
    private static final Angle L3Angle = Degrees.of(40.0) ;

    private static final Distance L2Dist = Centimeters.of(90.0) ;
    private static final Angle L2Angle = Degrees.of(40.0) ;

    private static final Distance L1Dist = Centimeters.of(70.0) ;
    private static final Angle L1Angle = Degrees.of(0.0) ;

    public ManualPlaceReadyCmd(ManipulatorSubsystem m, int level, boolean direct) {
        level_ = level ;
        m_ = m ;
    }

    @Override
    public void initialize() {
        switch(level_) {
            case 1:
                cmd_ = new GoToCmd(m_, L1Dist, L1Angle) ;
                break ;

            case 2:
                cmd_ = new GoToCmd(m_, L2Dist, L2Angle) ; 
                break ;

            case 3:
                cmd_ = new GoToCmd(m_, L3Dist, L3Angle) ;
                break ;

            case 4:
                cmd_ = new GoToCmd(m_, L4Dist, L4Angle) ;
                break ;
        }
        cmd_.schedule(); ;
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return cmd_.isFinished() ;
    }
}
