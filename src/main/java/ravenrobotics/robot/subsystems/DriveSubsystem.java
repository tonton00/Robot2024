package ravenrobotics.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

    //Encoders
    private final RelativeEncoder frontLeftEncoder = frontLeft.getEncoder();
    private final RelativeEncoder frontRightEncoder = frontRight.getEncoder();
    private final RelativeEncoder backLeftEncoder = backLeft.getEncoder();
    private final RelativeEncoder backRightEncoder = backRight.getEncoder();

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
    //Battery voltage
    private GenericEntry batteryVoltage = Telemetry.teleopTab.add("Battery Voltage", 12).getEntry();

    ///SysID
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> drivenDistance = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> distanceVelocity = mutable(RotationsPerSecond.of(0));

    private final SysIdRoutine.Mechanism sysIDMechanism = new SysIdRoutine.Mechanism(
        (Measure<Voltage> voltage) ->
        {
            frontLeft.set(voltage.in(Volts) / RobotController.getBatteryVoltage());
            frontRight.set(voltage.in(Volts) / RobotController.getBatteryVoltage());
            backLeft.set(voltage.in(Volts) / RobotController.getBatteryVoltage());
            backRight.set(voltage.in(Volts) / RobotController.getBatteryVoltage());
        },
        log -> 
        {
            log.motor("frontLeft")
                .voltage(appliedVoltage.mut_replace(frontLeft.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(drivenDistance.mut_replace(frontLeftEncoder.getPosition(), Rotations))
                .angularVelocity(distanceVelocity.mut_replace(frontLeftEncoder.getVelocity(), RotationsPerSecond));
            log.motor("frontRight")
                .voltage(appliedVoltage.mut_replace(frontLeft.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(drivenDistance.mut_replace(frontRightEncoder.getPosition(), Rotations))
                .angularVelocity(distanceVelocity.mut_replace(frontRightEncoder.getVelocity(), RotationsPerSecond));
            log.motor("backLeft")
                .voltage(appliedVoltage.mut_replace(backLeft.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(drivenDistance.mut_replace(backLeftEncoder.getPosition(), Rotations))
                .angularVelocity(distanceVelocity.mut_replace(backRightEncoder.getVelocity(), RotationsPerSecond));
            log.motor("backRight")
                .voltage(appliedVoltage.mut_replace(backRight.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(drivenDistance.mut_replace(backRightEncoder.getPosition(), Rotations))
                .angularVelocity(distanceVelocity.mut_replace(backRightEncoder.getVelocity(), RotationsPerSecond));
        }, this);

    private final SysIdRoutine sysIDRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        sysIDMechanism);


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
     * @param maxSpeed The current maximum speed of the drivetrain.
     */
    public void drive(ChassisSpeeds speeds, double maxSpeed)
    {
        MecanumDriveWheelSpeeds wheelSpeeds = KinematicsConstants.kDriveKinematics.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(maxSpeed);
        
        //Convert the speeds into the range for the motors, then set them.
        frontLeft.set(wheelSpeeds.frontLeftMetersPerSecond / DrivetrainConstants.kDriveMaxSpeedMPS);
        frontRight.set(wheelSpeeds.frontRightMetersPerSecond / DrivetrainConstants.kDriveMaxSpeedMPS);
        backLeft.set(wheelSpeeds.rearLeftMetersPerSecond / DrivetrainConstants.kDriveMaxSpeedMPS);
        backRight.set(wheelSpeeds.rearRightMetersPerSecond / DrivetrainConstants.kDriveMaxSpeedMPS);

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

    /**
     * Immediately stops all of the drive motors.
     */
    public void stopMotors()
    {
        frontLeft.stopMotor();
        frontRight.stopMotor();
        backLeft.stopMotor();
        backRight.stopMotor();
    }

    /**
     * Get the quasistatic SysID command.
     * @param direction The direction to run the motors.
     * @return The command to schedule.
     */
    public Command getSysIDQuasistatic(SysIdRoutine.Direction direction)
    {
        return sysIDRoutine.quasistatic(direction);
    }

    /**
     * Get the dynamic SysID command.
     * @param direction The direction to run the motors.
     * @return The command to schedule.
     */
    public Command getSysIDDynamic(SysIdRoutine.Direction direction)
    {
        return sysIDRoutine.dynamic(direction);
    }

    @Override
    public void periodic()
    {
        //Update the battery voltage on telemetry.
        batteryVoltage.setDouble(RobotController.getBatteryVoltage());
    }

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

    /**
     * Add the motors to the simulation (WIP).
     */
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
