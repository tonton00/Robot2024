package ravenrobotics.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase
{
    //Roller motor and encoder.
    private final CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.kRollerMotor, MotorType.kBrushless);
    private final RelativeEncoder rollerMotorEncoder = rollerMotor.getEncoder();

    private static IntakeSubsystem instance;

    /**
     * Returns the active instance of the IntakeSubsystem.
     * 
     * @return 
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

    private IntakeSubsystem()
    {
        //Configure the motors and encoders for use.
        configMotors();
        configEncoders();
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

        //Save configuration.
        rollerMotor.burnFlash();
    }

    /**
     * Configures the subsystem's encoders for use.
     */
    private void configEncoders()
    {
        rollerMotorEncoder.setPosition(0.0);
    }
}
