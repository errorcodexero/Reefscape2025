package frc.robot.commands.tests;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.DepositCoralCmd;
import frc.robot.subsystems.grabber.GrabberSubsystem ;
import frc.robot.subsystems.manipulator.ManipulatorGotoCmd;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class ManipulatorGotoAutoMode extends Command {
    private ManipulatorSubsystem m_ ;
    private GrabberSubsystem g_ ;

    static private ShuffleboardTab tab_ ;
    static private SimpleWidget elevator_ ;
    static private SimpleWidget arm_ ;
    static private SimpleWidget apply_ ;
    static private SimpleWidget grabber_ ;
    static private SimpleWidget arm_ready_ ;
    static private SimpleWidget elevator_ready_ ;

    private boolean last_apply_value_ = false;
    private boolean last_grabber_value_ = false;

    public ManipulatorGotoAutoMode(ManipulatorSubsystem m, GrabberSubsystem g) {
        m_ = m ;
        g_ = g ;
    }

    @Override
    public void initialize() {
        tab_ = Shuffleboard.getTab("ManipulatorGoto");

        if (elevator_ == null) {
            elevator_ = tab_.add("Elevator", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(0, 0)
                .withProperties(Map.of("min", 0.0, "max", 3.0)) ;
        }

        if (arm_ == null) {
            arm_ = tab_.add("Arm", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withSize(4, 1)
                .withPosition(0, 1)
                .withProperties(Map.of("min", 0.0, "max", 3.0 * Math.PI / 2.0 )) ;
        }

        if (apply_ == null) {
            apply_ = tab_.add("Apply", false).withWidget(BuiltInWidgets.kToggleSwitch).withSize(1, 1)
                .withPosition(0, 2);
        }

        if (grabber_ == null) {
            grabber_ = tab_.add("Grabber", false).withWidget(BuiltInWidgets.kToggleSwitch).withSize(1, 1)
                .withPosition(1, 2);
        }

        if (arm_ready_ == null) {
            arm_ready_ = tab_.add("Arm Ready", false).withWidget(BuiltInWidgets.kBooleanBox).withSize(1, 1)
                .withPosition(2, 2);
        }

        if (elevator_ready_ == null) {
            elevator_ready_ = tab_.add("Elevator Ready", false).withWidget(BuiltInWidgets.kBooleanBox).withSize(1, 1)
                .withPosition(3, 2);
        }
    }

    @Override
    public void execute() {
        if (shouldApplyNewSettings()) {
            double elevator = elevator_.getEntry().getDouble(0.0);
            double arm = arm_.getEntry().getDouble(0.0);
            ManipulatorGotoCmd cmd = new ManipulatorGotoCmd(m_, Meters.of(elevator), Radians.of(arm)) ;
            cmd.schedule() ;
        }
        
        if (shouldEjectCoral() && m_.isArmAtTarget() && m_.isElevatorAtTarget()) {
            DepositCoralCmd cmd = new DepositCoralCmd(g_) ;
            cmd.schedule();
        }

        if (m_.isArmAtTarget()) {
            arm_ready_.getEntry().setBoolean(true);
        } else {
            arm_ready_.getEntry().setBoolean(false);
        }

        if (m_.isElevatorAtTarget()) {
            elevator_ready_.getEntry().setBoolean(true);
        } else {
            elevator_ready_.getEntry().setBoolean(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private boolean shouldEjectCoral() {
        boolean curval = grabber_.getEntry().getBoolean(false);
        boolean ret = (curval == true) && (last_grabber_value_ == false);

        last_grabber_value_ = curval;

        return ret;
    } 

    private boolean shouldApplyNewSettings() {
        boolean curval = apply_.getEntry().getBoolean(false);
        boolean ret = (curval == true) && (last_apply_value_ == false);

        last_apply_value_ = curval;

        return ret;
    }    
}
