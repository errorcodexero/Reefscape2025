package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class TrackerIOQuest implements TrackerIO {

    private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("questnav");
    private final IntegerSubscriber miso = nt.getIntegerTopic("miso").subscribe(0);
    private final IntegerPublisher mosi = nt.getIntegerTopic("mosi").publish();

    private final DoubleSubscriber questTimestamp = nt.getDoubleTopic("timestamp").subscribe(0);
    private final DoubleSubscriber questBatteryPercent = nt.getDoubleTopic("device/batteryPercent").subscribe(0.0f);
    private final IntegerSubscriber questFrameCount = nt.getIntegerTopic("frameCount").subscribe(0);
    private final BooleanSubscriber questIsTracking = nt.getBooleanTopic("device/isTracking").subscribe(false);
    private final IntegerSubscriber questTrackingLostCount = nt.getIntegerTopic("device/trackingLostCounter").subscribe(0);
    private final FloatArraySubscriber questPosition = nt.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
    private final FloatArraySubscriber questQuaternion = nt.getFloatArrayTopic("quaternion").subscribe(new float[]{0.0f, 0.0f, 0.0f, 0.0f});

    @Override
    public void updateInputs(TrackerInputs inputs) {

        // Reset request on success to idle.
        if (miso.get() == 99) {
            mosi.set(0);
        }

        inputs.connected = Timer.getFPGATimestamp() - Microseconds.of(questBatteryPercent.get()).in(Seconds) < 0.25;
        inputs.timestamp = questTimestamp.get();
        inputs.batteryPercent = questBatteryPercent.get();
        inputs.frameCount = questFrameCount.get();
        inputs.isTracking = questIsTracking.get();
        inputs.trackingLostCount = questTrackingLostCount.get();

        inputs.pose = getPose3d();
        inputs.quaternion = getQuaternion();
    }

    @Override
    public void zeroHeading() {
        if (miso.get() != 99) {
            mosi.set(1);
        }
    }

    @Override
    public void zeroPosition() {
        // TODO Auto-generated method stub
        TrackerIO.super.zeroPosition();
    }

    private Pose3d getPose3d() {
        return new Pose3d(
            getTranslation3d(),
            getRotation3d()
        );
    }

    private Rotation3d getRotation3d() {
        return new Rotation3d(getQuaternion());
    }

    private Translation3d getTranslation3d() {
        float[] values = questPosition.get();
        return new Translation3d(
            values[2],
            -values[0], // Todo: why is this negative?
            values[1]
        );
    }

    private Quaternion getQuaternion() {
        float[] values = questQuaternion.get();

        return new Quaternion(
            values[0],
            values[1],
            values[2],
            values[3]
        );
    }
    
}