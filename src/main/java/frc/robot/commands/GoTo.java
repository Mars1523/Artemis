package frc.robot.commands;

import static frc.robot.Constants.Vision.kTagLayout;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.Set;

public class GoTo {

    private static final int redHub = 10;
    private static final int redReefNETagID = 9;
    private static final int redReefNWTagID = 11;
    private static final int redReefSTagID = 7;
    private static final int redReefSETagID = 8;
    private static final int redReefSWTagID = 6;
    private static final int redCsLeftTagID = 1;
    private static final int redCsRightTagID = 2;
    private static final int blueClimbTagID = 31;
    private static final int blueHub = 25;
    private static final int blueReefNETagID = 22;
    private static final int blueReefNWTagID = 20;
    private static final int blueReefSTagID = 18;
    private static final int blueReefSETagID = 17;
    private static final int blueReefSWTagID = 19;
    private static final int blueCsLeftTagID = 13;
    private static final int blueCsRightTagID = 12;
    private static final int redClimbTagID = 15;

    public static boolean isRed() {
        return getAlliance() == Alliance.Red;
    }

    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public static PathConstraints constraints = new PathConstraints(3.7, 3.9, 360 * 1.5, 360);

    private static Pose2d inFrontOfTag(int id) {
        Transform2d rot180 = new Transform2d(Translation2d.kZero, Rotation2d.k180deg);
        var tag = kTagLayout.getTagPose(id).get().toPose2d();
        var offset = new Transform2d(1.5, 0, new Rotation2d()); // 67
        Pose2d infrontOfTag = tag.plus(offset).transformBy(rot180);
        return infrontOfTag;
    }

    public static Command trench(SwerveDriveSubsystem swerveSub) {
        return Commands.defer(
                () -> {
                    Pose2d pose = Pose2d.kZero;
                    if (swerveSub.inNeutralZone()) {
                        if (swerveSub.inLeft()) {
                            pose = new Pose2d(3.6, 7.426, new Rotation2d(Math.PI));
                        } else {
                            pose = new Pose2d(3.6, 0.624, new Rotation2d(Math.PI));
                        }

                    } else {
                        if (swerveSub.inLeft()) {
                            pose = new Pose2d(5.7, 7.426, new Rotation2d(0));
                        } else {
                            pose = new Pose2d(5.7, 0.624, new Rotation2d(0));
                        }
                    }

                    if (getAlliance() == Alliance.Red) {
                        FlippingUtil.flipFieldPose(pose);
                    }
                    return AutoBuilder.pathfindToPose(pose, constraints);
                },
                Set.of(swerveSub));
    }

    public static Command climbLineUp() {
        return Commands.either(
                AutoBuilder.pathfindToPose(inFrontOfTag(redClimbTagID), constraints)
                        .alongWith(Commands.print("going to tag ID " + redClimbTagID)),
                AutoBuilder.pathfindToPose(inFrontOfTag(blueClimbTagID), constraints)
                        .alongWith(Commands.print("going to tag ID " + blueClimbTagID)),
                GoTo::isRed);
    }

    public static Command shootLineUp() {
        return Commands.either(
                AutoBuilder.pathfindToPose(inFrontOfTag(redHub), constraints)
                        .alongWith(Commands.print("going to tag ID " + redClimbTagID)),
                AutoBuilder.pathfindToPose(inFrontOfTag(blueHub), constraints)
                        .alongWith(Commands.print("going to tag ID " + blueClimbTagID)),
                GoTo::isRed);
    }
}
