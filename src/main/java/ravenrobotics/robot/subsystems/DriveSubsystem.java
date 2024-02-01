package ravenrobotics.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Robot;
import ravenrobotics.robot.Constants.DrivetrainConstants;
import ravenrobotics.robot.Constants.KinematicsConstants;
import ravenrobotics.robot.util.Telemetry;

public class DriveSubsystem extends SubsystemBase
{
    //Drivetrain motors. Configured for use with a Vortex motor, which is brushless.
    private final CANSparkMax frontLeft = new CANSparkMax(DrivetrainConstants.kFrontLeftMotor, MotorType.kBrushless);
    private final CANSparkMax frontRight = new CANSparkMax(DrivetrainConstants.kFrontRightMotor, MotorType.kBrushless);
    private final CANSparkMax backLeft = new CANSparkMax(DrivetrainConstants.kBackLeftMotor, MotorType.kBrushless);
    private final CANSparkMax backRight = new CANSparkMax(DrivetrainConstants.kBackRightMotor, MotorType.kBrushless);

    //Instance object for simplifying getting the subsystem for commands.
    private static DriveSubsystem instance;

    ///Shuffleboard (telemetry)
    //Target speeds
    private GenericEntry frontLeftTargetSpeed = Telemetry.teleopTab.add("FL Target Speed", 0).getEntry();
    private GenericEntry frontRightTargetSpeed = Telemetry.teleopTab.add("FR Target Speed", 0).getEntry();
    private GenericEntry backLeftTargetSpeed = Telemetry.teleopTab.add("BL Target Speed", 0).getEntry();
    private GenericEntry backRightTargetSpeed = Telemetry.teleopTab.add("BR Target Speed", 0).getEntry();
    //Target power
    private GenericEntry frontLeftPower = Telemetry.teleopTab.add("FL Target Power", 0).getEntry();
    private GenericEntry frontRightPower = Telemetry.teleopTab.add("FR Target Power", 0).getEntry();
    private GenericEntry backLeftPower = Telemetry.teleopTab.add("BL Target Power", 0).getEntry();
    private GenericEntry backRightPower = Telemetry.teleopTab.add("BR Target Power", 0).getEntry();

    private final MecanumDrive mecDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

    /**
     * Get the active instance of DriveSubsystem.
     * @return The DriveSubsystem instance.
     */
    public static DriveSubsystem getInstance()
    {
        //If the instance doesn't exist yet, create it.
        if (instance == null)
        {
            instance = new DriveSubsystem();
        }
        //Return the instance.
        return instance;
    }

    /**
     * Subsystem for controlling the drivetrain of the robot.
     */
    private DriveSubsystem()
    {
        //Configure the motors for use.
        configMotors();

        //Setup for simulation
        if(Robot.isSimulation())
        {
            addMotorsToSim();
        }
    }

    /**
     * Drive the drivetrain.
     * @param speeds The target speed of the drivetrain as a ChassisSpeeds object.
     */
    public void drive(ChassisSpeeds speeds)
    {
        MecanumDriveWheelSpeeds wheelSpeeds = KinematicsConstants.kDriveKinematics.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(DrivetrainConstants.kDriveMaxSpeedMPS);
        
        //Convert the speeds into the range for the motors, then set them.
        frontLeft.set(wheelSpeeds.frontLeftMetersPerSecond / DrivetrainConstants.kDriveMaxSpeedMPS);
        frontRight.set(wheelSpeeds.frontRightMetersPerSecond / DrivetrainConstants.kDriveMaxSpeedMPS);
        backLeft.set(wheelSpeeds.rearLeftMetersPerSecond / DrivetrainConstants.kDriveMaxSpeedMPS);
        backRight.set(wheelSpeeds.rearRightMetersPerSecond / DrivetrainConstants.kDriveMaxSpeedMPS);

        //Update the MecanumDrive object so that WPILib doesn't get angry :).
        mecDrive.feed();

        //Update Shuffleboard with all the target speeds.
        frontLeftTargetSpeed.setDouble(wheelSpeeds.frontLeftMetersPerSecond);
        frontRightTargetSpeed.setDouble(wheelSpeeds.frontRightMetersPerSecond);
        backLeftTargetSpeed.setDouble(wheelSpeeds.rearLeftMetersPerSecond);
        backRightTargetSpeed.setDouble(wheelSpeeds.rearRightMetersPerSecond);
        //Update Shuffleboard with powers.
        frontLeftPower.setDouble(frontLeft.get());
        frontRightPower.setDouble(frontRight.get());
        backLeftPower.setDouble(backLeft.get());
        backRightPower.setDouble(backRight.get());
    }

    public void driveWPI(double strafe, double forward, double rotation, Rotation2d angle, boolean isFieldRelative)
    {
        if (isFieldRelative)
        {
            mecDrive.driveCartesian(strafe, forward, rotation, angle);
        }
        else
        {
            mecDrive.driveCartesian(strafe, forward, rotation);
        }
    }

    @Override
    public void periodic()
    {}

    @Override
    public void simulationPeriodic()
    {
        //Update the BatterySim battery.
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            frontLeft.get() * DrivetrainConstants.kDriveMaxVoltage,
            frontRight.get() * DrivetrainConstants.kDriveMaxVoltage,
            backLeft.get() * DrivetrainConstants.kDriveMaxVoltage,
            backRight.get() * DrivetrainConstants.kDriveMaxVoltage);
    }
    
    public void addMotorsToSim()
    {
        REVPhysicsSim.getInstance().addSparkMax(frontLeft, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(frontRight, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(backLeft, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(backLeft, DCMotor.getNEO(1));
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
        frontLeft.setInverted(DrivetrainConstants.kInvertFrontLeftSide);
        backLeft.setInverted(DrivetrainConstants.kInvertBackLeftSide);
        frontRight.setInverted(DrivetrainConstants.kInvertFrontRightSide);
        backRight.setInverted(DrivetrainConstants.kInvertBackRightSide);

        //Set the ramp rate of the motors.
        frontLeft.setOpenLoopRampRate(DrivetrainConstants.kMotorRampRate);
        backLeft.setOpenLoopRampRate(DrivetrainConstants.kMotorRampRate);
        frontRight.setOpenLoopRampRate(DrivetrainConstants.kMotorRampRate);
        backRight.setOpenLoopRampRate(DrivetrainConstants.kMotorRampRate);

        //Set the idle mode to brake so that the robot does a better job of staying in place.
        frontLeft.setIdleMode(IdleMode.kBrake);
        frontRight.setIdleMode(IdleMode.kBrake);
        backLeft.setIdleMode(IdleMode.kBrake);
        backRight.setIdleMode(IdleMode.kBrake);

        //Save the configuration to the motors.
        frontLeft.burnFlash();
        backLeft.burnFlash();
        frontRight.burnFlash();
        backRight.burnFlash();
    }
}
