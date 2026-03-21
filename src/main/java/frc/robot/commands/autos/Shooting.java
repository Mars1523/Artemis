package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class Shooting extends SequentialCommandGroup {
    final SwerveDriveSubsystem swerveDriveSubsystem;
    final TurretSubsystem turretSubsystem;
    final LauncherSubsystem launcherSubsystem;

    public Shooting(
            SwerveDriveSubsystem swerveDriveSubsystem,
            TurretSubsystem turretSubsystem,
            LauncherSubsystem launcherSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.launcherSubsystem = launcherSubsystem;

        addCommands(
                turretSubsystem.setTurretAngleCommand(Rotation2d.fromDegrees(-4.07)),
                Commands.waitUntil(turretSubsystem::isTurretReady),
                launcherSubsystem.shootManually());
    }
}
