package ravenrobotics.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import ravenrobotics.robot.AutoConstants;
import ravenrobotics.robot.Robot;
import ravenrobotics.robot.Constants.DrivetrainConstants;
import ravenrobotics.robot.Constants.KinematicsConstants;
import ravenrobotics.robot.util.Telemetry;

public class DriveSubsystem extends SubsystemBase
{
    //Drivetrain motors. Configured for use with a NEO motor, which is brushless.
    private final CANSparkMax frontLeft = new CANSparkMax(DrivetrainConstants.kFrontLeftMotor, MotorType.kBrushless);
    private final CANSparkMax frontRight = new CANSparkMax(DrivetrainConstants.kFrontRightMotor, MotorType.kBrushless);
    private final CANSparkMax backLeft = new CANSparkMax(DrivetrainConstants.kBackLeftMotor, MotorType.kBrushless);
    private final CANSparkMax backRight = new CANSparkMax(DrivetrainConstants.kBackRightMotor, MotorType.kBrushless);

    //Drivetrain encoders.
    private final RelativeEncoder frontLeftEncoder = frontLeft.getEncoder();
    private final RelativeEncoder frontRightEncoder = frontRight.getEncoder();
    private final RelativeEncoder backLeftEncoder = backLeft.getEncoder();
    private final RelativeEncoder backRightEncoder = backRight.getEncoder();

    ///Shuffleboard (telemetry)
    //Target speeds
    private GenericEntry frontLeftTargetSpeed = Telemetry.teleopTab.add("FL Target Speed", 0).getEntry();
    private GenericEntry frontRightTargetSpeed = Telemetry.teleopTab.add("FR Target Speed", 0).getEntry();
    private GenericEntry backLeftTargetSpeed = Telemetry.teleopTab.add("BL Target Speed", 0).getEntry();
    private GenericEntry backRightTargetSpeed = Telemetry.teleopTab.add("BR Target Speed", 0).getEntry();

    //Target power
    private GenericEntry frontLeftPower = Telemetry.teleopTab.add("FL Power", 0).getEntry();
    private GenericEntry frontRightPower = Telemetry.teleopTab.add("FR Power", 0).getEntry();
    private GenericEntry backLeftPower = Telemetry.teleopTab.add("BL Power", 0).getEntry();
    private GenericEntry backRightPower = Telemetry.teleopTab.add("BR Power", 0).getEntry();
    
    //Battery voltage
    private GenericEntry batteryVoltage = Telemetry.teleopTab.add("Battery Voltage", 12).getEntry();

    //SysID variables for measuring the voltage, distance, and velocity.
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> drivenDistance = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> distanceVelocity = mutable(RotationsPerSecond.of(0));

    //SysID mechanism for applying the tests.
    private final SysIdRoutine.Mechanism sysIDMechanism = new SysIdRoutine.Mechanism(
        (Measure<Voltage> voltage) ->
        {
            //Set the motor power to the new voltages.
            frontLeft.set(voltage.in(Volts) / RobotController.getBatteryVoltage());
            frontRight.set(voltage.in(Volts) / RobotController.getBatteryVoltage());
            backLeft.set(voltage.in(Volts) / RobotController.getBatteryVoltage());
            backRight.set(voltage.in(Volts) / RobotController.getBatteryVoltage());
        },
        log -> 
        {
            //Log front left motor voltage, position, and velocity.
            log.motor("frontLeft")
                .voltage(appliedVoltage.mut_replace(frontLeft.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(drivenDistance.mut_replace(frontLeftEncoder.getPosition(), Rotations))
                .angularVelocity(distanceVelocity.mut_replace(frontLeftEncoder.getVelocity(), RotationsPerSecond));
            //Log front right motor voltage, position, and velocity.
            log.motor("frontRight")
                .voltage(appliedVoltage.mut_replace(frontLeft.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(drivenDistance.mut_replace(frontRightEncoder.getPosition(), Rotations))
                .angularVelocity(distanceVelocity.mut_replace(frontRightEncoder.getVelocity(), RotationsPerSecond));
            //Log back left motor voltage, position, and velocity.
            log.motor("backLeft")
                .voltage(appliedVoltage.mut_replace(backLeft.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(drivenDistance.mut_replace(backLeftEncoder.getPosition(), Rotations))
                .angularVelocity(distanceVelocity.mut_replace(backRightEncoder.getVelocity(), RotationsPerSecond));
            //Log back right motor voltage, position, and velocity.
            log.motor("backRight")
                .voltage(appliedVoltage.mut_replace(backRight.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(drivenDistance.mut_replace(backRightEncoder.getPosition(), Rotations))
                .angularVelocity(distanceVelocity.mut_replace(backRightEncoder.getVelocity(), RotationsPerSecond));
        }, this);

    //SysID Routine for creating the commands to run the routines.
    private final SysIdRoutine sysIDRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        sysIDMechanism);

    //Odometry-related things.
    private MecanumDriveOdometry driveOdometry;
    private Pose2d drivetrainPose;
    private Field2d fieldData = new Field2d();

    //Instance object for simplifying getting the subsystem for commands.
    private static DriveSubsystem instance;

    /**
     * Get the active instance of DriveSubsystem.
     * 
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
        //Configure the motors and encoders for use.
        configMotors();
        configEncoders();

        //Setup for simulation
        if(Robot.isSimulation())
        {
            addMotorsToSim();
        }
        
        //Add the Field2d widget to Shuffleboard so we can see the robot's position.
        Telemetry.teleopTab.add("Robot Position", fieldData);

        //Configure PathPlanner's AutoBuilder for use.
        AutoBuilder.configureHolonomic(
            //Gives the robot pose as a Pose2d.
            this::getRobotPose,
            //Resets the robot pose to a Pose2d (the beginning of the path).
            this::resetRobotPose,
            //Gets the robot speed so it can drive it per limitations.
            this::getRobotSpeed,
            //Feeds a ChassisSpeed object to the drivetrain for actually driving the path.
            this::drive,
            //Path following config.
            new HolonomicPathFollowerConfig(
                //PID Constants.
                AutoConstants.kAutoTranslationPIDConstants,
                AutoConstants.kAutoRotationPIDConstants,
                //Max speed in autonomous (m/s).
                AutoConstants.kMaxAutoSpeed,
                //The radius of the robot.
                AutoConstants.kRobotRadius,
                //Replanning config (what happens if the robot gets off of the path).
                new ReplanningConfig()
            ),
            //Check if the path should get flipped if we are on Red Alliance.
            () ->
            {
                //Get the current alliance.
                var alliance = DriverStation.getAlliance();
                //Check if there is an alliance present, then check if it is Red.
                if (alliance.isPresent())
                {
                    return alliance.get() == DriverStation.Alliance.Red;
                }

                return false;
            },

            //Add the subsystem as a requirement for the AutoBuilder commands.
            instance
        );
    }

    /**
     * Drive the drivetrain.
     * 
     * @param speeds The target speed of the drivetrain as a ChassisSpeeds object.
     * @param maxSpeed The current maximum speed of the drivetrain.
     */
    public void drive(ChassisSpeeds speeds, double maxSpeed)
    {
        System.out.println("Chassis Speeds: " + speeds);

        //TODO: Test Conditionals 
        //If we are primarliy rotating instead of driving, do a differential drive rotation.
        //# If twist/Z rotation value or converted rad value is greater than the converted X-value and the Y-value of the joystick 
       /* TEST 1
        if (Math.abs(speeds.omegaRadiansPerSecond) > speeds.vxMetersPerSecond && Math.abs(speeds.omegaRadiansPerSecond) > speeds.vyMetersPerSecond)
        {
            frontLeft.set(speeds.omegaRadiansPerSecond / maxSpeed);
            frontRight.set(speeds.omegaRadiansPerSecond / maxSpeed);
            backLeft.set(-speeds.omegaRadiansPerSecond / maxSpeed);
            backRight.set(-speeds.omegaRadiansPerSecond / maxSpeed);
            return;
        }
       */


        //TEST 2     Should do the same exact thing as test 1
        /* 
        if (Math.abs(speeds.omegaRadiansPerSecond) > speeds.vxMetersPerSecond)
        {
            frontLeft.set(speeds.omegaRadiansPerSecond / maxSpeed);
            frontRight.set(speeds.omegaRadiansPerSecond / maxSpeed);
            backLeft.set(-speeds.omegaRadiansPerSecond / maxSpeed);
            backRight.set(-speeds.omegaRadiansPerSecond / maxSpeed);
            return;
        }


       if (Math.abs(speeds.omegaRadiansPerSecond) > speeds.vyMetersPerSecond)
        {
            frontLeft.set(speeds.omegaRadiansPerSecond / maxSpeed);
            frontRight.set(speeds.omegaRadiansPerSecond / maxSpeed);
            backLeft.set(-speeds.omegaRadiansPerSecond / maxSpeed);
            backRight.set(-speeds.omegaRadiansPerSecond / maxSpeed);
            return;
        }
        */
                //Test Three
                //if twist is grater than or equal to x |logical operatior or| twist is greater than or equal two y
        /* 
         if (Math.abs(speeds.omegaRadiansPerSecond) >= speeds.vxMetersPerSecond ||  Math.abs(speeds.omegaRadiansPerSecond) >= speeds.vyMetersPerSecond)
        {
            frontLeft.set(speeds.omegaRadiansPerSecond / maxSpeed);
            frontRight.set(speeds.omegaRadiansPerSecond / maxSpeed);
            backLeft.set(-speeds.omegaRadiansPerSecond / maxSpeed);
            backRight.set(-speeds.omegaRadiansPerSecond / maxSpeed);
            return;
        }
        */
        // Test Four
         // If the JoystickValue for Z isnt zero then rotate
        /* 
         if (Joystick.AxisType.kZ.value != 0) 
         {
           frontLeft.set(speeds.omegaRadiansPerSecond / maxSpeed);
            frontRight.set(speeds.omegaRadiansPerSecond / maxSpeed);
            backLeft.set(-speeds.omegaRadiansPerSecond / maxSpeed);
            backRight.set(-speeds.omegaRadiansPerSecond / maxSpeed);
            return;
         }
        */
        //Last Test
         /* 
         if (Joystick.AxisType.kZ.value > speeds.vxMetersPerSecond && Joystick.AxisType.kZ.value > speeds.vyMetersPerSecond) 
         {
           frontLeft.set(speeds.omegaRadiansPerSecond / maxSpeed);
            frontRight.set(speeds.omegaRadiansPerSecond / maxSpeed);
            backLeft.set(-speeds.omegaRadiansPerSecond / maxSpeed);
            backRight.set(-speeds.omegaRadiansPerSecond / maxSpeed);
            return;
         }
         */







        //Convert the ChassisSpeeds to individual wheel speeds.
        MecanumDriveWheelSpeeds wheelSpeeds = KinematicsConstants.kDriveKinematics.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(maxSpeed);
        
        //Convert the speeds into the range for the motors, then set them.
        frontLeft.set(wheelSpeeds.frontLeftMetersPerSecond / maxSpeed);
        frontRight.set(wheelSpeeds.frontRightMetersPerSecond / maxSpeed);
        backLeft.set(wheelSpeeds.rearLeftMetersPerSecond / maxSpeed);
        backRight.set(wheelSpeeds.rearRightMetersPerSecond / maxSpeed);

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
     * Drive the drivetrain.
     * 
     * @param speeds The target speeds of the drivetrain as a ChassisSpeeds object.
     */
    public void drive(ChassisSpeeds speeds)
    {
        //Convert the chassis speeds to wheel speeds and make sure they aren't overshooting our max speed we can actually drive.
        MecanumDriveWheelSpeeds wheelSpeeds = KinematicsConstants.kDriveKinematics.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(DrivetrainConstants.kDriveMaxSpeedMPS);

        //Set the motors to the speeds.
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
     * Returns the robot's speed as a ChassisSpeeds.
     * 
     * @return The robot's speed as a ChassisSpeeds.
     */
    public ChassisSpeeds getRobotSpeed()
    {
        //Create a MecanumDriveWheelSpeeds object from the encoder speeds.
        var speeds = new MecanumDriveWheelSpeeds(
            frontLeftEncoder.getVelocity(),
            frontRightEncoder.getVelocity(),
            backLeftEncoder.getVelocity(),
            backRightEncoder.getVelocity()
        );

        return KinematicsConstants.kDriveKinematics.toChassisSpeeds(speeds);
    }

    /**
     * Returns the positions of the drivetrain wheels.
     * 
     * @return The positions of the wheels as a MecanumDriveWhelPositions.
     */
    public MecanumDriveWheelPositions getWheelPositions()
    {
        return new MecanumDriveWheelPositions(
            frontLeftEncoder.getPosition(),
            frontRightEncoder.getPosition(),
            backLeftEncoder.getPosition(),
            backRightEncoder.getPosition()
        );
    }

    /**
     * Returns the drivetrain's pose as a Pose2d.
     * 
     * @return The drivetrain's pose as a Pose2d.
     */
    public Pose2d getRobotPose()
    {
        return drivetrainPose;
    }

    /**
     * Resets the robot's odometry to a new position.
     * 
     * @param newPose The robot's new position as a Pose2d.
     */
    public void resetRobotPose(Pose2d newPose)
    {
        drivetrainPose = newPose;
        driveOdometry.resetPosition(newPose.getRotation(), new MecanumDriveWheelPositions(), newPose);
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
     * 
     * @param direction The direction to run the motors.
     * @return The command to schedule.
     */
    public Command getSysIDQuasistatic(SysIdRoutine.Direction direction)
    {
        return sysIDRoutine.quasistatic(direction);
    }

    /**
     * Get the dynamic SysID command.
     * 
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

        //Update the odometry.
        drivetrainPose = driveOdometry.update(
            IMUSubsystem.getInstance().getYaw(),
            getWheelPositions()
        );

        //Update the robot pose on the field widget.
        fieldData.setRobotPose(drivetrainPose);
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

    private void configEncoders()
    {
        //Set positions to 0.
        frontLeftEncoder.setPosition(0.0);
        frontRightEncoder.setPosition(0.0);
        backLeftEncoder.setPosition(0.0);
        backRightEncoder.setPosition(0.0);

        //Sets the velocity conversion factor to give us m/s.
        frontLeftEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
        frontRightEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
        backLeftEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
        backRightEncoder.setVelocityConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
        //Sets the position factor to give us accurate distance measurements.
        frontLeftEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
        frontRightEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
        backLeftEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);
        backRightEncoder.setPositionConversionFactor(DrivetrainConstants.kEncoderConversionFactor);

        //Initialize odometry since encoders are ready.
        driveOdometry = new MecanumDriveOdometry(
            KinematicsConstants.kDriveKinematics,
            IMUSubsystem.getInstance().getYaw(), 
            getWheelPositions());
    }
}
