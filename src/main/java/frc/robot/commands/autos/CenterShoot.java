package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class CenterShoot extends SequentialCommandGroup {
    final SwerveDriveSubsystem swerveDriveSubsystem;
    final TurretSubsystem turretSubsystem;
    final LauncherSubsystem launcherSubsystem;
    final IntakeSubsystem intakeSubsystem;

    public CenterShoot(
            SwerveDriveSubsystem swerveDriveSubsystem,
            TurretSubsystem turretSubsystem,
            LauncherSubsystem launcherSubsystem,
            IntakeSubsystem intakeSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.launcherSubsystem = launcherSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(turretSubsystem, launcherSubsystem);
        try {
            addCommands(
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("CenterShootAutoPath")),
                    intakeSubsystem.intakeDown(),
                    turretSubsystem.setTurretAngleCommand(Rotation2d.fromDegrees(-4.07)),
                    Commands.waitUntil(turretSubsystem::isTurretReady),
                    new ParallelCommandGroup(
                            intakeSubsystem.runIntake(),
                            launcherSubsystem
                                    .shootDistanceCommand(Meters.of(3.156))
                                    .withTimeout(10)));
        } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}
