// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import ravenrobotics.robot.Constants.DriverStationConstants;
import ravenrobotics.robot.commands.DriveCommand;
import ravenrobotics.robot.subsystems.DriveSubsystem;

public class RobotContainer 
{
  //Drive subsystem for driving the drivebase.
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  //Driver controller (drives the robot around).
  private final CommandXboxController driverController = new CommandXboxController(DriverStationConstants.kDriverPort);

  //Whether to drive field relative or not.
  private boolean isFieldRelative = true;

  //Main drive command.
  private final DriveCommand driveCommand = new DriveCommand(
    driveSubsystem,
    () -> -driverController.getLeftX(),
    () -> -driverController.getLeftY(),
    () -> -driverController.getLeftX(),
    () -> isFieldRelative);

  public RobotContainer() 
  {
    //Set the default command to the drive command so the robot can always drive.
    driveSubsystem.setDefaultCommand(driveCommand);
    //Configure configured controller bindings.
    configureBindings();
  }

  private void configureBindings()
  {
    //Set the X button on the driver controller to toggle whether we are driving field relative.
    driverController.x().onTrue(new InstantCommand(() -> toggleFieldRelative()));
  }

  private void toggleFieldRelative()
  {
    //Toggle field relative (if true set false, if false set true)
    if (isFieldRelative)
    {
      isFieldRelative = false;
    }
    else
    {
      isFieldRelative = true;
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
