package frc.team670.mustanglib.utils.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class MustangPose2d extends Pose2d {
    @Override
    public Twist2d log(Pose2d end) {
        final var transform = end.relativeTo(this);
        final var dtheta = transform.getRotation().getRadians();
        final var halfDtheta = dtheta / 2.0;

        final var cosMinusOne = transform.getRotation().getCos() - 1;

        double halfThetaByTanOfHalfDtheta;
        if (Math.abs(cosMinusOne) < 1E-9) {
        halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
        halfThetaByTanOfHalfDtheta = -(halfDtheta * transform.getRotation().getSin()) / cosMinusOne;
        }

        Translation2d translationPart =
            transform
                .getTranslation()
                .rotateBy(new Rotation2d(halfThetaByTanOfHalfDtheta, -halfDtheta));

        return new Twist2d(translationPart.getX(), translationPart.getY(), dtheta);
    }

    
    
}
