package ravenrobotics.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot.subsystems.FlywheelSubsystem;
import ravenrobotics.robot.subsystems.IntakeSubsystem;

public class RunFlywheelCommand extends Command 
{
    private final IntakeSubsystem intakeSubsystem;
    private final FlywheelSubsystem flywheelSubsystem;
    
    public RunFlywheelCommand()
    {
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.flywheelSubsystem = FlywheelSubsystem.getInstance();

        addRequirements(intakeSubsystem, flywheelSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        flywheelSubsystem.shootOn();
        Timer.delay(1);
        intakeSubsystem.runRollersSlow();
        Timer.delay(0.025);
        intakeSubsystem.runRollers();
    }

    @Override
    public void end(boolean interrupted)
    {
        flywheelSubsystem.stopFly();
        intakeSubsystem.stopRollers();
    }
}
