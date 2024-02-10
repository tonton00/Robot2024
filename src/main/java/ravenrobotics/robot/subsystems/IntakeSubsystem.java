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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase
{
    //Roller motor and encoder.
    private final CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.kRollerMotor, MotorType.kBrushless);
    private final RelativeEncoder rollerMotorEncoder = rollerMotor.getEncoder();
    //Arm motor and encoder.
    private final CANSparkMax armMotor = new CANSparkMax(IntakeConstants.kArmMotor, MotorType.kBrushless);
    private final RelativeEncoder intakeArmMotorEncoder = armMotor.getEncoder();

    //PID Controller for the arm.
    private final SparkPIDController armPIDController = armMotor.getPIDController();
    //Controllers for the rollers.
    private final BangBangController rollerBBController = new BangBangController();
    private final SparkPIDController rollerPIDController = rollerMotor.getPIDController();

    //Distance sensor.
    //private final Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

    private static IntakeSubsystem instance;

    public enum IntakeArmPosition
    {
        kDeployed,
        kRetracted
    }

    private final Thread rollerThread = new Thread(() ->
    {
        //TODO: Get the distance sensors so we can actually build this.
        boolean isLoaded = false;
        while (!isLoaded)
        {
            rollerPIDController.setReference(rollerBBController.calculate(rollerMotorEncoder.getVelocity(), IntakeConstants.kRollerSetpoint), ControlType.kVelocity);
            //Add things.
        }
        rollerMotor.stopMotor();
    });

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
    }

    /**
     * Run the full intake routine.
     */
    public void runIntakeRoutine()
    {
        setIntakePosition(IntakeArmPosition.kDeployed);
        rollerThread.start();
        //Join the thread and wait.
        try {
            rollerThread.join();
        } catch (InterruptedException e) {
            System.out.println("Roller thread canceled.");
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
        if (position == IntakeArmPosition.kDeployed)
        {
            //If we want to deploy the arm, set it to it's reference point.
            armPIDController.setReference(IntakeConstants.kArmDeployedSetpoint, ControlType.kPosition);
        }
        else if (position == IntakeArmPosition.kRetracted)
        {
            //If we want to retract the arm
            armPIDController.setReference(0, ControlType.kPosition);
        }
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

        //Set the PID constants for the PID controller.
        armPIDController.setP(IntakeConstants.kArmP);
        armPIDController.setI(IntakeConstants.kArmI);
        armPIDController.setD(IntakeConstants.kArmD);

        armPIDController.setFeedbackDevice(intakeArmMotorEncoder);

        //Save configuration.
        rollerMotor.burnFlash();
    }

    /**
     * Configures the subsystem's encoders for use.
     */
    private void configEncoders()
    {
        rollerMotorEncoder.setPosition(0.0);
        intakeArmMotorEncoder.setPosition(0.0);
    }
}
