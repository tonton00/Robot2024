package ravenrobotics.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants.DrivetrainConstants;
import ravenrobotics.robot.Constants.KinematicsConstants;
import ravenrobotics.robot.util.Conversions;

public class DriveSubsystem extends SubsystemBase
{
    //Drivetrain motors. Configured for use with a NEO motor, which is brushless.
    private final CANSparkMax frontLeft = new CANSparkMax(DrivetrainConstants.kFrontLeftMotor, MotorType.kBrushless);
    private final CANSparkMax frontRight = new CANSparkMax(DrivetrainConstants.kFrontRightMotor, MotorType.kBrushless);
    private final CANSparkMax backLeft = new CANSparkMax(DrivetrainConstants.kBackLeftMotor, MotorType.kBrushless);
    private final CANSparkMax backRight = new CANSparkMax(DrivetrainConstants.kBackRightMotor, MotorType.kBrushless);

    /**
     * Subsystem for controlling the drivetrain of the robot.
     */
    public DriveSubsystem()
    {
        //Configure the motors for use.
        configMotors();
    }

    /**
     * Drive the drivetrain.
     * @param speeds The target speed of the drivetrain as a ChassisSpeeds object.
     */
    public void drive(ChassisSpeeds speeds)
    {
        MecanumDriveWheelSpeeds wheelSpeeds = KinematicsConstants.kDriveKinematics.toWheelSpeeds(speeds);
        //TODO: Figure out the max speed of real robot.
        wheelSpeeds.desaturate(DrivetrainConstants.kDriveMaxSpeedMPS);
        
        //Convert the speeds into voltages and apply them to the motors.
        frontLeft.setVoltage(Conversions.MPSToVoltage(wheelSpeeds.frontLeftMetersPerSecond, DrivetrainConstants.kDriveMaxVoltage));
        frontRight.setVoltage(Conversions.MPSToVoltage(wheelSpeeds.frontRightMetersPerSecond, DrivetrainConstants.kDriveMaxVoltage));
        backLeft.setVoltage(Conversions.MPSToVoltage(wheelSpeeds.rearLeftMetersPerSecond, DrivetrainConstants.kDriveMaxVoltage));
        backRight.setVoltage(Conversions.MPSToVoltage(wheelSpeeds.rearRightMetersPerSecond, DrivetrainConstants.kDriveMaxVoltage));
    }

    /**
     * Configure the drivetrain motors for use.
     */
    private void configMotors()
    {
        //Restore all the motors to factory defaults, so that we can start fresh and nothing interferes.
        frontLeft.restoreFactoryDefaults();
        frontRight.restoreFactoryDefaults();
        backLeft.restoreFactoryDefaults();
        backRight.restoreFactoryDefaults();

        //Reverse the default direction of the left side so everything drives normally.
        frontLeft.setInverted(DrivetrainConstants.invertLeftSide);
        backLeft.setInverted(DrivetrainConstants.invertLeftSide);
        frontRight.setInverted(DrivetrainConstants.invertRightSide);
        backRight.setInverted(DrivetrainConstants.invertRightSide);
    }
}
