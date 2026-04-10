package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AimingSub;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class CenterShoot extends SequentialCommandGroup {
    final SwerveDriveSubsystem swerveDriveSubsystem;
    final AimingSub aimingSub;
    final IntakeSubsystem intakeSubsystem;

    public CenterShoot(
            SwerveDriveSubsystem swerveDriveSubsystem, AimingSub aimingSub, IntakeSubsystem intakeSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.aimingSub = aimingSub;
        this.intakeSubsystem = intakeSubsystem;

        try {
            addCommands(
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("CenterShootAutoPath"))
                            .withTimeout(3),
                    intakeSubsystem.intakeDown().withTimeout(0.5),
                    new ParallelRaceGroup(aimingSub.shootPhotonCommand().withTimeout(10), intakeSubsystem.runIntake()));

        } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}
