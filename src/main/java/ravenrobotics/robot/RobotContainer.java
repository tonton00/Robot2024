// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import ravenrobotics.robot.Constants.DriverStationConstants;
import ravenrobotics.robot.commands.DriveCommand;
import ravenrobotics.robot.subsystems.DriveSubsystem;
import ravenrobotics.robot.subsystems.IMUSubsystem;
import ravenrobotics.robot.util.Telemetry;

public class RobotContainer 
{
  //Driver controller (drives the robot around).
  //private final CommandXboxController driverController = new CommandXboxController(DriverStationConstants.kDriverPort);
  private final CommandJoystick driverJoystick = new CommandJoystick(DriverStationConstants.kDriverPort);

  //Whether to drive field relative or not.
  public boolean isFieldRelative = false;
  private GenericEntry isFieldRelativeEntry = Telemetry.teleopTab.add("Field Relative", false).getEntry();

  //Main drive command.
  private final DriveCommand driveCommand = new DriveCommand(
    () -> -driverJoystick.getX(),
    () -> -driverJoystick.getY(),
    () -> -driverJoystick.getZ(),
    () -> -driverJoystick.getThrottle(),
    () -> isFieldRelative);

  public RobotContainer()
  {
    isFieldRelativeEntry.setBoolean(isFieldRelative);
    //Configure configured controller bindings.
    configureBindings();
    DriveSubsystem.getInstance().setDefaultCommand(driveCommand);
  }

  private void configureBindings()
  {
    //Set the buttons on the joystick for field-relative and zeroing the heading.
    driverJoystick.button(2).onTrue(new InstantCommand(() -> toggleFieldRelative()));
    driverJoystick.button(3).onTrue(new InstantCommand(() -> IMUSubsystem.getInstance().zeroYaw()));
  }

  private void toggleFieldRelative()
  {
    //Toggle field relative (if true set false, if false set true)
    if (isFieldRelative)
    {
      isFieldRelative = false;
      isFieldRelativeEntry.setBoolean(isFieldRelative);
    }
    else
    {
      isFieldRelative = true;
      isFieldRelativeEntry.setBoolean(isFieldRelative);
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
