package ravenrobotics.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot.Constants.DrivetrainConstants;
import ravenrobotics.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command
{
    //The drive subsystem so the drivetrain can be driven ;).
    private final DriveSubsystem driveSubsystem;
    
    //Suppliers for joystick values and whether to drive field relative.
    private final DoubleSupplier xSpeed, ySpeed, tSpeed;
    private final BooleanSupplier isFieldRelative;

    //Limiters so we don't break the chassis by instantly applying power.
    private final SlewRateLimiter xLimiter, yLimiter, tLimiter;

    /**
     * Command to drive the robot using joystick axes.
     * @param driveSubsystem The drivetrain subsystem.
     * @param strafeSpeed The axis for moving left/right.
     * @param forwardSpeed The axis for moving forward/backward.
     * @param rotationSpeed The axis for rotating.
     * @param isFieldRelative The boolean for whether to drive field relative.
     */
    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier strafeSpeed, DoubleSupplier forwardSpeed, DoubleSupplier rotationSpeed, BooleanSupplier isFieldRelative)
    {
        //Initialize subsystem instance.
        this.driveSubsystem = driveSubsystem;

        //Initialize double suppliers (getting joystick values)
        this.xSpeed = strafeSpeed;
        this.ySpeed = forwardSpeed;
        this.tSpeed = rotationSpeed;

        //Initialize boolean supplier (whether to drive field or robot relative)
        this.isFieldRelative = isFieldRelative;

        //Initialize SlewRateLimiters so we don't acclerate too quickly.
        xLimiter = new SlewRateLimiter(0.02);
        yLimiter = new SlewRateLimiter(0.02);
        tLimiter = new SlewRateLimiter(0.02);

        //Add the subsystem as a requirement for the command, so the subsystem isn't being controlled by two different commands at once.
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize()
    {}

    @Override
    public void execute()
    {
        //Temporary variables for the speeds.
        double xSpeedMPS, ySpeedMPS, tSpeedMPS;

        //Get the target strafe, forward/backward, and rotation speeds.
        xSpeedMPS = xLimiter.calculate(xSpeed.getAsDouble()) * DrivetrainConstants.kDriveMaxSpeedMPS;
        ySpeedMPS = yLimiter.calculate(ySpeed.getAsDouble()) * DrivetrainConstants.kDriveMaxSpeedMPS;
        tSpeedMPS = tLimiter.calculate(tSpeed.getAsDouble()) * DrivetrainConstants.kDriveMaxSpeedMPS;

        ChassisSpeeds targetSpeeds = new ChassisSpeeds(xSpeedMPS, ySpeedMPS, tSpeedMPS);

        //Convert the chassis speeds if driving field-oriented.
        if (isFieldRelative.getAsBoolean())
        {
            //TODO: IMU implementaiton
            targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, null);
        }

        //Drive the subsystem.
        driveSubsystem.drive(targetSpeeds);
    }
}
