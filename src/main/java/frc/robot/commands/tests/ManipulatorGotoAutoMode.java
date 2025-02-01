package frc.robot.commands.tests;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.ManipulatorGotoCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class ManipulatorGotoAutoMode extends Command {
    private ManipulatorSubsystem m_ ;

    static private ShuffleboardTab tab_ ;
    static private SimpleWidget elevator_ ;
    static private SimpleWidget arm_ ;
    static private SimpleWidget apply_ ;

    private boolean last_apply_value_ = false;

    public ManipulatorGotoAutoMode(ManipulatorSubsystem m) {
        m_ = m ;
    }

    @Override
    public void initialize() {
        tab_ = Shuffleboard.getTab("ManipulatorGoto");

        if (elevator_ == null) {
            elevator_ = tab_.add("Elevator", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(2, 1)
                .withPosition(0, 0)
                .withProperties(Map.of("min", 0.0, "max", 3.0)) ;
        }

        if (arm_ == null) {
            arm_ = tab_.add("Arm", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(2, 1)
                .withPosition(0, 1)
                .withProperties(Map.of("min", 0.0, "max", 270.0)) ;
        }

        if (apply_ == null) {
            apply_ = tab_.add("Apply", false).withWidget(BuiltInWidgets.kToggleSwitch).withSize(1, 1)
                .withPosition(0, 2);
        }
    }

    @Override
    public void execute() {
        if (shouldApplyNewSettings()) {
            double elevator = elevator_.getEntry().getDouble(0.0);
            double arm = arm_.getEntry().getDouble(0.0);
            ManipulatorGotoCmd cmd = new ManipulatorGotoCmd(m_, Meters.of(elevator), Degrees.of(arm)) ;
            cmd.schedule() ;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private boolean shouldApplyNewSettings() {
        boolean curval = apply_.getEntry().getBoolean(false);
        boolean ret = (curval == true) && (last_apply_value_ == false);

        last_apply_value_ = curval;

        return ret;
    }    
}
