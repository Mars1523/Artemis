// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DefaultSwerve;
import frc.robot.commands.autos.AutoRotate;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
    CommandJoystick primaryJoy = new CommandJoystick(0);
    SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    DefaultSwerve defaultSwerve = new DefaultSwerve(primaryJoy.getHID(), swerveDriveSubsystem);

    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    public RobotContainer() {
        swerveDriveSubsystem.setDefaultCommand(defaultSwerve);
        configureAutos();
        configureBindings();
    }

    private void configureAutos() {
        autoChooser.addOption("Rotate", new AutoRotate(swerveDriveSubsystem, 45, 0.1));
        Shuffleboard.getTab("auto").add(autoChooser);
    }

    private void configureBindings() {
        primaryJoy.button(12).whileTrue(swerveDriveSubsystem.resetJoystickForwardAngle());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
