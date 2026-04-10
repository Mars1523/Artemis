package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AimingSub;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class NeutralLeft extends SequentialCommandGroup {
    final SwerveDriveSubsystem swerveSubsystem;
    final IntakeSubsystem intakeSubsystem;
    final AimingSub aimingSub;

    public NeutralLeft(SwerveDriveSubsystem swerveSubsystem, AimingSub aimingSub, IntakeSubsystem intakeSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.aimingSub = aimingSub;
        this.intakeSubsystem = intakeSubsystem;

            addCommands(
                    aimingSub.shootPhotonCommand().withTimeout(6),
                    new PathPlannerAuto("NeutralLeftAuto"));
    
}
}
