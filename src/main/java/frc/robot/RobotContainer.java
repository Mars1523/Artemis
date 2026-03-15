// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultSwerve;
import frc.robot.commands.autos.AutoRotate;
import frc.robot.subsystems.AimingSub;
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
    //         new RobotOrientedControllerSwerve(commandXboxController.getHID(), swerveDriveSubsystem);

    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    FuelInputSubsystem fuelInputSubsystem = new FuelInputSubsystem();
    LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
    TurretSubsystem turretSubsystem = new TurretSubsystem(swerveDriveSubsystem);
    // AimingSub aimSub = new AimingSub(swerveDriveSubsystem, turretSubsystem,
    // launcherSubsystem);
    PhotonCameraSubsystem photonCameraSubsystem = new PhotonCameraSubsystem(
            swerveDriveSubsystem::acceptVisionData,
            () -> swerveDriveSubsystem.getRobotVelocity().omegaRadiansPerSecond);

    AimingSub aimingSub = new AimingSub(swerveDriveSubsystem, turretSubsystem, launcherSubsystem);

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
        // todo: add climb
        // map primaryJoy 8 to climb up
        // map primaryJoy 7 to climb down

        primaryJoy.button(12).whileTrue(swerveDriveSubsystem.resetJoystickForwardAngle());
        commandXboxController.a().whileTrue(fuelInputSubsystem.runIntake());
        commandXboxController.b().whileTrue(aimingSub.shootPhotonCommand());
        commandXboxController.y().whileTrue(launcherSubsystem.shootManually());
        commandXboxController.povUp().onTrue(fuelInputSubsystem.intakeUp());
        commandXboxController.povDown().onTrue(fuelInputSubsystem.intakeDown());

        Command rotateTurretCommand = Commands.run(
                () -> {
                    double input =
                            -commandXboxController.getLeftTriggerAxis() + commandXboxController.getRightTriggerAxis();
                    turretSubsystem.rotateTurret(input);
                },
                turretSubsystem);
        commandXboxController.rightTrigger(0.05).whileTrue(rotateTurretCommand);
        commandXboxController.leftTrigger(0.05).whileTrue(rotateTurretCommand);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
