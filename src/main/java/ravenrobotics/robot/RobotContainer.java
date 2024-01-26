// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import ravenrobotics.robot.Constants.DriverStationConstants;
import ravenrobotics.robot.commands.DriveCommand;
import ravenrobotics.robot.commands.DriveCommandWPI;
import ravenrobotics.robot.subsystems.DriveSubsystem;
import ravenrobotics.robot.subsystems.IMUSubsystem;
import ravenrobotics.robot.util.Telemetry;

public class RobotContainer 
{
  //Driver controller (drives the robot around).
  private final CommandXboxController driverController = new CommandXboxController(DriverStationConstants.kDriverPort);

  //Whether to drive field relative or not.
  public boolean isFieldRelative = false;
  private GenericEntry isFieldRelativeEntry = Telemetry.teleopTab.add("Field Relative", false).getEntry();
  private final SendableChooser<Command> chooser = new SendableChooser<Command>();

  //Main drive command.
  private final DriveCommand driveCommand = new DriveCommand(
    () -> -driverController.getLeftX(),
    () -> -driverController.getLeftY(),
    () -> -driverController.getRightX(),
    () -> isFieldRelative);

  private final DriveCommandWPI otherDriveCommand = new DriveCommandWPI(
    () -> -driverController.getLeftX(),
    () -> -driverController.getLeftY(),
    () -> -driverController.getRightX(),
    () -> isFieldRelative);

  public RobotContainer()
  {
    chooser.addOption("Custom", driveCommand);
    chooser.addOption("WPI", otherDriveCommand);
    Telemetry.teleopTab.add("Drive Command", chooser);
    isFieldRelativeEntry.setBoolean(isFieldRelative);
    //Configure configured controller bindings.
    configureBindings();
  }

  private void configureBindings()
  {
    //Set the X button on the driver controller to toggle whether we are driving field relative.
    driverController.x().onTrue(new InstantCommand(() -> toggleFieldRelative()));
    driverController.y().onTrue(new InstantCommand(() -> IMUSubsystem.getInstance().zeroYaw()));
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

  public void setDriveCommand()
  {
    Command option = chooser.getSelected();
    DriveSubsystem.getInstance().setDefaultCommand(option);
  }
}
