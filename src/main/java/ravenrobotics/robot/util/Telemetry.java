package ravenrobotics.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Telemetry
{
    public static ShuffleboardTab teleopTab = Shuffleboard.getTab("TeleOp");
    public static String teleopTabString = "TeleOp";

    public static void switchToTeleopTab()
    {
        Shuffleboard.selectTab(teleopTabString);
    }
}
