package frc.robot.commands.tests;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.ManipulatorGotoCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class ManualPlaceReadyCmd extends Command {
    private int level_ ;
    private ManipulatorSubsystem m_ ;
    private ManipulatorGotoCmd cmd_ ;
    private boolean direct_ ;

    private static final Distance L4Dist = Centimeters.of(214.0) ;
    private static final Angle L4Angle = Degrees.of(60.0) ;

    private static final Distance L3Dist = Centimeters.of(214.0) ;
    private static final Angle L3Angle = Degrees.of(60.0) ;

    private static final Distance L2Dist = Centimeters.of(214.0) ;
    private static final Angle L2Angle = Degrees.of(60.0) ;

    private static final Distance L1Dist = Centimeters.of(214.0) ;
    private static final Angle L1Angle = Degrees.of(60.0) ;

    public ManualPlaceReadyCmd(ManipulatorSubsystem m, int level, boolean direct) {
        level_ = level ;
        m_ = m ;
        direct_ = direct ;
    }

    @Override
    public void initialize() {
        switch(level_) {
            case 1:
                cmd_ = new ManipulatorGotoCmd(m_, L1Dist, L1Angle, direct_) ;
                break ;

            case 2:
                cmd_ = new ManipulatorGotoCmd(m_, L2Dist, L2Angle, direct_) ; 
                break ;

            case 3:
                cmd_ = new ManipulatorGotoCmd(m_, L3Dist, L3Angle, direct_) ;
                break ;

            case- 4:
                cmd_ = new ManipulatorGotoCmd(m_, L4Dist, L4Angle, direct_) ;
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
