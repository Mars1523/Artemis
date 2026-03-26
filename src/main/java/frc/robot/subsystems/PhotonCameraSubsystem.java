package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Did not implement yet...
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Manages a Photon Camera and returns information about detected apriltags.
 * To configure the camera, connect to robot wifi and open in url: http://photonvision.local:5800/
 * See here for info on the url: https://docs.photonvision.org/en/latest/docs/quick-start/networking.html
 * The camera should be calibrated in the url before use, but only needs to be done once.
 * Make sure the camera name in the url matches the camera name in the PhotonCamera constructor.
 */
public class PhotonCameraSubsystem extends SubsystemBase {
    private static class Cam {
        final PhotonCamera camera;
        final PhotonPoseEstimator estimator;

        Cam(String name, PhotonPoseEstimator estimator) {
            this.camera = new PhotonCamera(name);
            this.estimator = estimator;
        }
    }

    private final List<Cam> cams = new ArrayList<>();
    private final EstimateConsumer estConsumer;
    private final DoubleSupplier yawRateRadPerSec;

    private final double maxPoseAmbiguity = 0.2;
    private final double maxYawRate = 3.5;

    private final double worstcaseFPS = 27;
    private Matrix<N3, N1> curStdDevs = kSingleTagStdDevs;
    private Optional<PhotonPipelineResult> lastResult = Optional.empty();
    private double lastResultTimestamp = 0;

    private final PhotonPipelineResult kEmptyPipelineResult = new PhotonPipelineResult();

    public PhotonCameraSubsystem(EstimateConsumer estConsumer, DoubleSupplier yawRateRadPerSec) {
        this.estConsumer = estConsumer;
        this.yawRateRadPerSec = yawRateRadPerSec;

        cams.add(new Cam("leftCamera", new PhotonPoseEstimator(kTagLayout, kRobotToCameraLeft)));
        cams.add(new Cam("rightCamera", new PhotonPoseEstimator(kTagLayout, kRobotToCameraRight)));
        // Replace Camera with whatever name you want. Theoretically you could also add more cameras
    }

    @Override
    public void periodic() {

        if (Math.abs(yawRateRadPerSec.getAsDouble()) > maxYawRate) {
            return;
        }

        List<Pose2d> poses = new ArrayList<>();
        double newestTs = 0.0;

        for (Cam cam : cams) {
            var results = cam.camera.getAllUnreadResults();
            for (var result : results) {
                // No targets
                if (!result.hasTargets()) continue;
                // If ALL are too ambiguous
                if (!hasLowAmb(result, maxPoseAmbiguity)) continue;

                Optional<EstimatedRobotPose> visionEst = estimatedPose(cam.estimator, result);
                Logger.recordOutput(
                        "Photon/" + cam.camera.getName() + "EstimatedPose3D", visionEst.get().estimatedPose);
                updateEstimationStdDevs(visionEst, result.getTargets());

                poses.add(visionEst.get().estimatedPose.toPose2d());
                newestTs = Math.max(newestTs, visionEst.get().timestampSeconds);

                lastResult = Optional.of(result);
                lastResultTimestamp = Timer.getFPGATimestamp();

                // visionEst.ifPresent(est -> {
                //     // Change our trust in the measurement based on the tags we can see
                //     var estStdDevs = getEstimationStdDevs();

                //     estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                // });
            }
        }

        if (poses.isEmpty()) {
            return;
        }

        double x = 0, y = 0, s = 0, c = 0;
        for (Pose2d p : poses) {
            x += p.getX();
            y += p.getY();
            s += Math.sin(p.getRotation().getRadians());
            c += Math.cos(p.getRotation().getRadians());
        }
        x /= poses.size();
        y /= poses.size();
        Rotation2d averageRotation = new Rotation2d(Math.atan2(s / poses.size(), c / poses.size()));
        Pose2d averagePose = new Pose2d(x, y, averageRotation);

        var estimationStdDevs = getEstimationStdDevs();
        estConsumer.accept(averagePose, newestTs, estimationStdDevs);

        //     if (results.size() > 0) {
        //         var result = results.get(results.size() - 1);
        //         if (result.hasTargets()) {
        //             lastResult = Optional.of(result);
        //             lastResultTimestamp = Timer.getFPGATimestamp();
        //         }
        //     }
        // }

        Logger.recordOutput("Photon/EstimatedPose2D", averagePose);
        Logger.recordOutput("Photon/EstimationStdDevs", estimationStdDevs);
    }

    /**
     * Returns the most recent valid results
     */
    public PhotonPipelineResult getLastResult() {
        if (lastResultTimestamp < Timer.getFPGATimestamp() - (1.0 / worstcaseFPS)) {
            return kEmptyPipelineResult;
        }
        return lastResult.orElse(kEmptyPipelineResult);
    }

    private static boolean hasLowAmb(PhotonPipelineResult result, double maxAmb) {
        for (PhotonTrackedTarget t : result.getTargets()) {
            if (t.getFiducialId() < 0) continue; // if its not an april tag
            double amb = t.getPoseAmbiguity();
            if (amb >= 0 && amb <= maxAmb) {
                return true;
            }
        }
        return false;
    }

    public static Optional<EstimatedRobotPose> estimatedPose(
            PhotonPoseEstimator estimator, PhotonPipelineResult result) {
        Optional<EstimatedRobotPose> est = estimator.estimateCoprocMultiTagPose(result);
        if (est.isEmpty()) {
            est = estimator.estimateLowestAmbiguityPose(result);
        }
        return est;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = kTagLayout.getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist += tagPose.get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(
                                estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
