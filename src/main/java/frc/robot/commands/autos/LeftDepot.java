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

public class LeftDepot extends SequentialCommandGroup {
    SwerveDriveSubsystem swerveDriveSubsystem;
    IntakeSubsystem intakeSubsystem;
    AimingSub aimingSub;

    public LeftDepot(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem, AimingSub aimingSub) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.aimingSub = aimingSub;

        try {
            addCommands(
                    intakeSubsystem.intakeDown(),
                    new ParallelRaceGroup(
                            AutoBuilder.followPath(PathPlannerPath.fromPathFile("DepotLeft")),
                            aimingSub.shootPhotonCommand(),
                            intakeSubsystem.runIntake()),
                    // shootphoton and intake is run not run once
                    // how to find file path
                    aimingSub.shootPhotonCommand().withTimeout(5));
        } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}
