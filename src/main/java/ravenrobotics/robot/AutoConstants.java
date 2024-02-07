package ravenrobotics.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.util.Units;

public class AutoConstants 
{
    public static final PIDConstants kAutoTranslationPIDConstants = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants kAutoRotationPIDConstants = new PIDConstants(5.0, 0.0, 0.0);

    public static final double kMaxAutoSpeed = 5;
    public static final double kRobotRadius = Units.inchesToMeters(15);
}
