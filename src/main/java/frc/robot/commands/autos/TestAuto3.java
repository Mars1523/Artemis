package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class TestAuto3 extends SequentialCommandGroup {
    SwerveDriveSubsystem swerveDriveSubsystem;
    IntakeSubsystem intakeSubsystem;

    public TestAuto3(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        try {
            addCommands(new ParallelRaceGroup(
                    intakeSubsystem.runIntake(), AutoBuilder.followPath(PathPlannerPath.fromPathFile("TestAutoPath"))));
        } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}
