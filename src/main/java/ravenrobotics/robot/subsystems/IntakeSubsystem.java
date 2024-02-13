package ravenrobotics.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants.IntakeConstants;
import ravenrobotics.robot.util.Telemetry;

public class IntakeSubsystem extends SubsystemBase
{
    //Roller motor and encoder.
    private final CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.kRollerMotor, MotorType.kBrushless);
    private final RelativeEncoder rollerMotorEncoder = rollerMotor.getEncoder();
    //Arm motor and encoder.
    private final CANSparkMax armMotor = new CANSparkMax(IntakeConstants.kArmMotor, MotorType.kBrushless);
    private final RelativeEncoder armMotorEncoder = armMotor.getEncoder();

    //PID Controller for the arm.
    private final SparkPIDController armPIDController = armMotor.getPIDController();
    //Controllers for the rollers.
    private final BangBangController rollerBBController = new BangBangController();
    private final SparkPIDController rollerPIDController = rollerMotor.getPIDController();

    //Distance sensor.
    //private final Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

    //Shuffleboard
    private final GenericEntry armPositionEntry = Telemetry.teleopTab.add("Arm Position", 0).getEntry();

    private static IntakeSubsystem instance;

    public enum IntakeArmPosition
    {
        kDeployed,
        kRetracted
    }

    private final Command rollerCommand = new Command()
    {
        boolean isLoaded = false;
        @Override
        public void execute()
        {
            rollerPIDController.setReference(rollerBBController.calculate(rollerMotorEncoder.getVelocity(), IntakeConstants.kRollerSetpoint), ControlType.kVelocity);
            //TODO: Get measurement from lasers.
            double noteDistance = 50;
            if (noteDistance < IntakeConstants.kNoteInDistance)
            {
                isLoaded = true;
            }
        }

        @Override
        public void end(boolean isInterrupted)
        {
            rollerMotor.stopMotor();
        }

        @Override
        public boolean isFinished()
        {
            return isLoaded;
        }
    };

    /**
     * Returns the active instance of the IntakeSubsystem.
     * 
     * @return The IntakeSubsystem instance.
     */
    public static IntakeSubsystem getInstance()
    {
        //If the instance hasn't been created it yet, create it.
        if (instance == null)
        {
            instance = new IntakeSubsystem();
        }
        
        //Return the instance.
        return instance;
    }

    /**
     * Subsystem for controlling the intake on the robot.
     */
    private IntakeSubsystem()
    {
        //Configure the motors and encoders for use.
        configMotors();
        configEncoders();

        rollerCommand.addRequirements(this);
    }

    /**
     * Run the full intake routine.
     */
    public void runIntakeRoutine()
    {
        setIntakePosition(IntakeArmPosition.kDeployed);
        rollerCommand.schedule();
        while (rollerCommand.isScheduled())
        {
            continue;
        }
        setIntakePosition(IntakeArmPosition.kRetracted);
    }

    /**
     * Sets the intake position.
     * 
     * @param position The desired position of the intake.
     */
    public void setIntakePosition(IntakeArmPosition position)
    {
        switch(position)
        {
            case kDeployed -> armPIDController.setReference(IntakeConstants.kArmDeployedSetpoint, ControlType.kSmartMotion);
            case kRetracted -> armPIDController.setReference(0, ControlType.kSmartMotion);
        }
    }

    public void runRollers()
    {
        rollerMotor.set(1);
    }

    public void stopRollers()
    {
        rollerMotor.stopMotor();
    }

    /**
     * Interrupts (cancels) the roller thread.
     */
    public void cancelWaitRoutine()
    {
        rollerCommand.cancel();
    }

    @Override
    public void periodic()
    {
        //Update arm motor's position on Shuffleboard.
        armPositionEntry.setDouble(armMotorEncoder.getPosition());
    }

    /**
     * Configures the subsystem's motors for use.
     */
    private void configMotors()
    {
        //Reset to factory defaults.
        rollerMotor.restoreFactoryDefaults();

        //Set idle mode.
        rollerMotor.setIdleMode(IdleMode.kCoast);
        armMotor.setIdleMode(IdleMode.kCoast);

        //Set the PID constants for the PID controller.
        armPIDController.setP(IntakeConstants.kArmP);
        armPIDController.setI(IntakeConstants.kArmI);
        armPIDController.setD(IntakeConstants.kArmD);

        armPIDController.setFeedbackDevice(armMotorEncoder);

        //Save configuration.
        rollerMotor.burnFlash();
    }

    /**
     * Configures the subsystem's encoders for use.
     */
    private void configEncoders()
    {
        rollerMotorEncoder.setPosition(0.0);
        armMotorEncoder.setPosition(0.0);
    }
}
