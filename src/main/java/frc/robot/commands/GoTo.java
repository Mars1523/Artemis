package frc.robot.commands;

import static frc.robot.Constants.Vision.kTagLayout;

import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class GoTo {

    private final static int redReefNTagID = 10;
    private final static int redReefNETagID = 9;
    private final static int redReefNWTagID = 11;
    private final static int redReefSTagID = 7;
    private final static int redReefSETagID = 8;
    private final static int redReefSWTagID = 6;
    private final static int redCsLeftTagID = 1;
    private final static int redCsRightTagID = 2;
    private final static int redProcessorTagID = 3;
    private final static int blueReefNTagID = 21;
    private final static int blueReefNETagID = 22;
    private final static int blueReefNWTagID = 20;
    private final static int blueReefSTagID = 18;
    private final static int blueReefSETagID = 17;
    private final static int blueReefSWTagID = 19;
    private final static int blueCsLeftTagID = 13;
    private final static int blueCsRightTagID = 12;
    private final static int blueProcessorTagID = 16;

    public static boolean isRed() {
        return getAlliance() == Alliance.Red;
    }

    public static Alliance getAlliance() {
        System.out
                .println("returning alliance: " + DriverStation.getAlliance());
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public static PathConstraints constraints =
            new PathConstraints(3.7, 3.9, 360 * 1.5, 360);

    private static Pose2d inFrontOfTag(int id) {
        Transform2d rot180 =
                new Transform2d(Translation2d.kZero, Rotation2d.k180deg);
        var tag = kTagLayout.getTagPose(id).get().toPose2d();
        var offset = new Transform2d(1.2, 0, new Rotation2d());
        Pose2d infrontOfTag = tag.plus(offset).transformBy(rot180);
        return infrontOfTag;
    }

    
    public static Command processor() {
        return Commands.either(AutoBuilder
                .pathfindToPose(inFrontOfTag(redProcessorTagID), constraints)
                .alongWith(
                        Commands.print("going to tag ID " + redProcessorTagID)),
                AutoBuilder
                        .pathfindToPose(inFrontOfTag(blueProcessorTagID),
                                constraints)
                        .alongWith(Commands.print(
                                "going to tag ID " + blueProcessorTagID)),
                GoTo::isRed);
    }


}
