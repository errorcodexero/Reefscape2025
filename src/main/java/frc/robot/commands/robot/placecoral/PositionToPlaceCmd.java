package frc.robot.commands.robot.placecoral;

import static edu.wpi.first.units.Units.Milliseconds;

import org.littletonrobotics.junction.Logger;
import org.xerosw.util.XeroTimer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorConstants.Arm;
import frc.robot.subsystems.manipulator.ManipulatorConstants.Elevator;
import frc.robot.RobotContainer;
import frc.robot.Constants.ReefLevel;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.manipulator.commands.GoToCmd;

public class PositionToPlaceCmd extends Command {
	private static final boolean kSkipDistanceChecks = false ;
	private static final boolean kSkipAngleChecks = false ;

	private enum State {
		Idle,
		Moving,
		Waiting,
		Done,
	}

	private final ManipulatorSubsystem m_;
    private final Drive db_ ;
	private final BrainSubsystem b_ ;
	private final GrabberSubsystem g_ ;
	private final Pose2d target_pose_;
	private final ReefLevel level_;
	private Command cmd_;
	private Distance target_elev_pos_;
	private Angle target_arm_pos_;
	private XeroTimer timer_ ;
	private State state_ ;

	public PositionToPlaceCmd(Drive db, BrainSubsystem b, ManipulatorSubsystem m, GrabberSubsystem g, ReefLevel level, Pose2d target) {
		addRequirements(m);
		m_ = m;
        db_ = db ;
		b_ = b;
		g_ = g ;
        
		level_ = level;
        target_pose_ = target;
	}

	@Override
	public void initialize() {
        int coral = findCoralOnFloor() ;
		target_arm_pos_ = null ;
		target_elev_pos_ = null ;

        switch(coral) {
			case -1:
				enableGamePad();
				b_.coralOnFloor() ;
				b_.clearRobotActions() ;
				break ;

            case 1:
				if (level_ == ReefLevel.L4) {
					enableGamePad();
					b_.coralOnFloor() ;
					b_.clearRobotActions();
				}
				else {
	                oneCoralOnFloor() ;
				}
                break ;

			case 0:	
            default:
                noCoralOnFloor() ;
                break ;
        }

		if (target_arm_pos_ != null && target_elev_pos_ != null) {
			cmd_ = new GoToCmd(m_, target_elev_pos_, target_arm_pos_) ;
			cmd_.initialize() ;
			state_ = State.Moving ;
		}
		else {
			state_ = State.Done ;
		}
	}

	private void enableGamePad() {
		RobotContainer.getInstance().gamepad().setLocked(false) ;
	}

	private void noCoralOnFloor() {
		timer_ = null ;
		switch (level_) {
			case L1:
				target_elev_pos_ = Elevator.Positions.kPlaceL1;
				target_arm_pos_ = Arm.Positions.kPlaceL1;
				break;

			case L2:
				target_elev_pos_ = Elevator.Positions.kPlaceL2;
				target_arm_pos_ = Arm.Positions.kPlaceL2;
				break;

			case L3:
				target_elev_pos_ = Elevator.Positions.kPlaceL3;
				target_arm_pos_ = Arm.Positions.kPlaceL3;
				break;

			case L4:
				target_elev_pos_ = Elevator.Positions.kPlaceL4;
				target_arm_pos_ = Arm.Positions.kPlaceL4;
				break;

			default:
				// Just to keep the intellisense happy
				break;
		}
	}

	private void oneCoralOnFloor() {
		timer_ = new XeroTimer(Milliseconds.of(500)) ;
		switch (level_) {
			case L1:
				target_elev_pos_ = Elevator.Positions.kPlaceL1;
				target_arm_pos_ = Arm.Positions.kPlaceL1;
				break;

			case L2:
				target_elev_pos_ = Elevator.Positions.kPlaceL2.plus(Elevator.Positions.kPlaceL2OneCoralAdder);
				target_arm_pos_ = Arm.Positions.kPlaceL2;
				break;

			case L3:
				target_elev_pos_ = Elevator.Positions.kPlaceL3.plus(Elevator.Positions.kPlaceL3OneCoralAdder);
				target_arm_pos_ = Arm.Positions.kPlaceL3;
				break;

			case L4:
				break;

			default:
				// Just to keep the intellisense happy
				break;
		}
	}

    private int findCoralOnFloor() {
		int ret = 0 ;

		if (!kSkipAngleChecks) {
			double delta = target_pose_.getRotation().minus(db_.getPose().getRotation()).getDegrees() ;
			if (Math.abs(delta) > 5.0 && level_ == ReefLevel.L4) {
				Logger.recordOutput("place/status", "abort-angle-" + delta) ;
				ret = -1 ;
			}
			else if (Math.abs(delta) > 5.0 && (level_ == ReefLevel.L2 || level_ == ReefLevel.L3)) {
				Logger.recordOutput("place/status", "abort-angle-" + delta) ;
				ret = -1 ;
			}
			else {
				Logger.recordOutput("place/status", "none") ;
			}
		}

		if (ret == 0 && !kSkipDistanceChecks) {
			ret = g_.coralOnFloor() ;
			if (ret > 1) {
				ret = -1 ;
			}
		}

		Logger.recordOutput("place/findCoralOnFloor", ret) ;
        return ret ;
    }

	@Override
	public void execute() {
		
		switch(state_) {
			case Moving:
				cmd_.execute();
				if (cmd_.isFinished()) {
					if (timer_ != null) {
						state_ = State.Waiting ;
						timer_.start() ;
					}
					else {
						state_ = State.Done ;
					}
				}
				break ;

			case Waiting:
				if (timer_.isExpired()) {
					state_ = State.Done ;
				}
				break ;

			case Idle:
			case Done:
				break ;
		}
	}

	@Override
	public boolean isFinished() {
        return state_ == State.Done ;
	}

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            cmd_.cancel() ;
        }
		if (cmd_ != null) {
	        cmd_.end(interrupted);
		}
    }
}
