// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DefaultSwerve;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
    CommandJoystick primaryJoy = new CommandJoystick(0);
    SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    DefaultSwerve defaultSwerve = new DefaultSwerve(primaryJoy.getHID(), swerveDriveSubsystem);

    public RobotContainer() {
        swerveDriveSubsystem.setDefaultCommand(defaultSwerve);
        configureBindings();
    }

    private void configureBindings() {
        primaryJoy.button(12).whileTrue(swerveDriveSubsystem.resetJoystickForwardAngle());
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
