// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.input.XboxInputImplementation;
import frc.lib.logging.interfaces.Loggerable;
import frc.robot.commands.drive.DriveWithHeadingCommand;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class RobotContainer implements Loggerable{
  // --- Robot inputs ---
  private final XboxInputImplementation primaryController = new XboxInputImplementation(0);

  // --- Drive train system ---
  private final DrivetrainSubsystem drivetrainSubsystem = SubsystemFactory.createDrivetrainSubsystem();
  private final DriveWithHeadingCommand drivetrainDefaultCommand = new DriveWithHeadingCommand(
    drivetrainSubsystem, 
    primaryController::getThrottle,   // throttle
    primaryController::getStrafe,     // strafe
    primaryController::getRotation,   // turn
    0.05,             // Chassis translational speed threshold
    0.05,              // Chassis rotational speed threshold
    0.05,                             // Drive joystick deadband
    0.05,                                 // Steer joystick deadband
    3.6,                                 // The max drive speed of the robot in meters per second
    8.2                                 // The max angular rate of the robot in Radians per second
  );

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Set default drive train command
    drivetrainSubsystem.setDefaultCommand(drivetrainDefaultCommand);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
