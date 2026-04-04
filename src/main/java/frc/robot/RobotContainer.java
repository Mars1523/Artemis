// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultSwerve;
import frc.robot.commands.autos.LeftDepot;
import frc.robot.commands.autos.Shooting;
import frc.robot.commands.autos.TestAuto3;
import frc.robot.subsystems.AimingSub;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PhotonCameraSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class RobotContainer {

    CommandJoystick primaryJoy = new CommandJoystick(0);
    CommandXboxController commandXboxController = new CommandXboxController(1);
    SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();

    // change to RobotOrientedControllerSwerve if preferred for testing
    DefaultSwerve defaultSwerve = new DefaultSwerve(primaryJoy.getHID(), swerveDriveSubsystem);
    // RobotOrientedControllerSwerve defaultSwerve =
    //         new RobotOrientedControllerSwerve(commandXboxController.getHID(), swerveDriveSubsystem);

    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
    TurretSubsystem turretSubsystem = new TurretSubsystem();
    // Climb climbSubsystem = new Climb();
    // AimingSub aimSub = new AimingSub(swerveDriveSubsystem, turretSubsystem,
    // launcherSubsystem);
    PhotonCameraSubsystem photonCameraSubsystem = new PhotonCameraSubsystem(
            swerveDriveSubsystem::acceptVisionData,
            () -> swerveDriveSubsystem.getRobotVelocity().omegaRadiansPerSecond);

    AimingSub aimingSub = new AimingSub(swerveDriveSubsystem, turretSubsystem, launcherSubsystem);

    public RobotContainer() {
        swerveDriveSubsystem.setDefaultCommand(defaultSwerve);

        NamedCommands.registerCommand("shootVelocityCommand", launcherSubsystem.shootVelocityCommand());
        NamedCommands.registerCommand("intakeUp", intakeSubsystem.intakeUp());
        NamedCommands.registerCommand("intakeDown", intakeSubsystem.intakeDown());
        NamedCommands.registerCommand("runIntake", intakeSubsystem.runIntake());
        NamedCommands.registerCommand("runIntakeReverse", intakeSubsystem.runIntakeReverse());
        NamedCommands.registerCommand("shootAtHomeCommand", aimingSub.shootPhotonCommand());
        NamedCommands.registerCommand("shootAtHubCommand", aimingSub.shootPhotonCommand());
        configureAutos();
        configureBindings();
    }

    private void configureAutos() {
        Shuffleboard.getTab("auto").add(autoChooser);
        // autoChooser.addOption("Climb", new climbAuto(climbSubsystem, swerveDriveSubsystem));
        autoChooser.addOption("LeftDepotReal", new LeftDepot(swerveDriveSubsystem, intakeSubsystem, aimingSub));
    }

    private void configureBindings() {
        // todo: add climb (statud: done)
        // map primaryJoy 8 to climb up
        // map primaryJoy 7 to climb down

        primaryJoy.button(12).whileTrue(swerveDriveSubsystem.resetJoystickForwardAngle());
        // primaryJoy.button(8).whileTrue(climbSubsystem.armUpCommand());
        // primaryJoy.button(7).whileTrue(climbSubsystem.armDownCommand());
        // primaryJoy.button(9).whileTrue(climbSubsystem.armUpCommand2()); 9 WORKS, 11 only drive, 10 only intake, 8
        // only drive
        // primaryJoy.button(10).whileTrue(climbSubsystem.armDownCommand2());
        var testAuto = new PathPlannerAuto("TestShoot");
        testAuto.event("shootAtHub").whileTrue(aimingSub.shootPhotonCommand());

        primaryJoy.button(11).onTrue(testAuto);

        primaryJoy.button(10).onTrue(new PathPlannerAuto("TestAuto2"));

        primaryJoy.button(9).onTrue(new TestAuto3(swerveDriveSubsystem, intakeSubsystem));

        try {
            primaryJoy.button(8).onTrue(AutoBuilder.followPath(PathPlannerPath.fromPathFile("TestAutoPath")));
        } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        // primaryJoy.button(6).onTrue(new climbAuto(climbSubsystem, swerveDriveSubsystem));

        commandXboxController.a().whileTrue(intakeSubsystem.runIntake());
        commandXboxController.b().whileTrue(aimingSub.shootPhotonCommand());
        commandXboxController.y().whileTrue(launcherSubsystem.shootManually());

        // for unsticking balls (potentially - not sure if needed)
        commandXboxController.x().whileTrue(launcherSubsystem.reverseHopper());

        // intake up needs to wait for turret to point forwards
        commandXboxController
                .povUp()
                .onTrue(Commands.sequence(
                        turretSubsystem.setTurretAngleCommand(new Rotation2d(0)),
                        Commands.waitUntil(
                                () -> Math.abs(turretSubsystem.getTurretAngle().getRotations()) < 0.02),
                        intakeSubsystem.intakeUp()));

        commandXboxController.povDown().onTrue(intakeSubsystem.intakeDown());

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
        var command = autoChooser.getSelected();
        if (command == null) {
            return Commands.none();
        }
        return command;
    }
}
