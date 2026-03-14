// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultSwerve;
import frc.robot.commands.autos.AutoRotate;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.FuelInputSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PhotonCameraSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
    CommandJoystick primaryJoy = new CommandJoystick(0);
    CommandXboxController commandXboxController = new CommandXboxController(1);
    SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();

    // change to RobotOrientedControllerSwerve if preferred for testing
    DefaultSwerve defaultSwerve = new DefaultSwerve(primaryJoy.getHID(), swerveDriveSubsystem);
    // RobotOrientedControllerSwerve defaultSwerve =
    //        new RobotOrientedControllerSwerve(commandXboxController.getHID(), swerveDriveSubsystem);

    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    FuelInputSubsystem fuelInputSubsystem = new FuelInputSubsystem();
    LauncherSubsystem launcherSubsystem = new LauncherSubsystem();

    TurretSubsystem turretSubsystem = new TurretSubsystem(swerveDriveSubsystem);

    AimingSubsystem aimingSubsystem = new AimingSubsystem(swerveDriveSubsystem, launcherSubsystem, turretSubsystem);

    PhotonCameraSubsystem photonCameraSubsystem = new PhotonCameraSubsystem(
            swerveDriveSubsystem::acceptVisionData,
            () -> swerveDriveSubsystem.getRobotVelocity().omegaRadiansPerSecond);

    public RobotContainer() {
        swerveDriveSubsystem.setDefaultCommand(defaultSwerve);
        NamedCommands.registerCommand("shootVelocityCommand", launcherSubsystem.shootVelocityCommand());
        NamedCommands.registerCommand("intakeUp", fuelInputSubsystem.intakeUp());
        NamedCommands.registerCommand("intakeDown", fuelInputSubsystem.intakeDown());
        NamedCommands.registerCommand("runIntake", fuelInputSubsystem.runIntake());
        NamedCommands.registerCommand("runIntakeReverse", fuelInputSubsystem.runIntakeReverse());
        NamedCommands.registerCommand("shootAtHomeCommand", turretSubsystem.shootAtHomeCommand());
        NamedCommands.registerCommand("shootAtHubCommand", turretSubsystem.shootAtHubCommand());
        configureAutos();
        configureBindings();
    }

    private void configureAutos() {
        autoChooser.addOption("Rotate", new AutoRotate(swerveDriveSubsystem, 45, 0.1));
        Shuffleboard.getTab("auto").add(autoChooser);
    }

    private void configureBindings() {
        primaryJoy.button(12).whileTrue(swerveDriveSubsystem.resetJoystickForwardAngle());
        commandXboxController.a().whileTrue(fuelInputSubsystem.runIntake());
        commandXboxController.b().whileTrue(launcherSubsystem.shootPhotonCommand());
        commandXboxController.x().whileTrue(launcherSubsystem.shootVelocityCommand());
        commandXboxController.rightBumper().onTrue(fuelInputSubsystem.intakeUp());
        commandXboxController.leftBumper().onTrue(fuelInputSubsystem.intakeDown());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
