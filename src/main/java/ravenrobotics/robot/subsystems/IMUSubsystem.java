package ravenrobotics.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants.IMUConstants;

public class IMUSubsystem extends SubsystemBase 
{
    //Pigeon2 object for actually interfacing with the IMU.
    private final Pigeon2 imu = new Pigeon2(IMUConstants.kPigeon2ID);

    //Instance object for simplifying getting the subsystem in commands.
    private static IMUSubsystem instance;

    /**
     * Get the active instance of the IMUSubsystem.
     * @return The active IMUSubsystem instance.
     */
    public static IMUSubsystem getInstance()
    {
        //Create the instance if it doesn't exist yet.
        if (instance == null)
        {
            instance = new IMUSubsystem();
        }
        //Return the instance.
        return instance;
    }

    /**
     * Private initializer for the subsystem; configures the IMU settings then zeros the yaw.
     */
    private IMUSubsystem()
    {
        //Configure IMU.
        configIMU();
        //Zero the heading.
        zeroYaw();
    }

    /**
     * Get the current heading of the IMU.
     * @return The heading as a Rotation2d.
     */
    public Rotation2d getYaw()
    {
        var heading = imu.getYaw().refresh().getValueAsDouble();
        return Rotation2d.fromDegrees(heading);
    }

    /**
     * Set the heading of the IMU to zero.
     */
    public void zeroYaw()
    {
        imu.setYaw(0.0);
    }

    /**
     * Configures the IMU for use.
     */
    private void configIMU()
    {
        //Factory reset the Pigeon2.
        imu.getConfigurator().apply(new Pigeon2Configuration());
        var config = new Pigeon2Configuration();

        //Disable using newer (potentially unsupported) configs by default.
        config.FutureProofConfigs = false;
        //IMU trim.
        config.GyroTrim.GyroScalarX = IMUConstants.kTrimX;
        config.GyroTrim.GyroScalarY = IMUConstants.kTrimY;
        config.GyroTrim.GyroScalarZ = IMUConstants.kTrimZ;
        //IMU pose.
        config.MountPose.MountPosePitch = IMUConstants.kMountPitch;
        config.MountPose.MountPoseRoll = IMUConstants.kMountRoll;
        config.MountPose.MountPoseYaw = IMUConstants.kMountYaw;
        //IMU features on/off.
        config.Pigeon2Features.DisableNoMotionCalibration = IMUConstants.kDisableNoMotionCalibration;
        config.Pigeon2Features.DisableTemperatureCompensation = IMUConstants.kDisableTemperatureCompensation;
        //Keep this to true, disables the whole purpose of the IMU otherwise :D.
        config.Pigeon2Features.EnableCompass = true;

        //Apply the config.
        imu.getConfigurator().apply(config);
    }
}
