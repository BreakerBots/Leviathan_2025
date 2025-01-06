package frc.robot.BreakerLib.swerve;

import java.util.function.BiConsumer;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.util.logging.BreakerLog;

//TODO switch to field rel version of ApplyChassisSpeeds when support for module force FF is added
public class BreakerSwerveChoreoController implements BiConsumer<Pose2d, SwerveSample> {
    private final BreakerSwerveDrivetrain drivetrain;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;
    private final SwerveRequest.ApplyFieldSpeeds request;

    public BreakerSwerveChoreoController(
        BreakerSwerveDrivetrain drivetrain,
        PIDController xController,
        PIDController yController,
        PIDController thetaController) {
        this.drivetrain = drivetrain;
        this.xController = xController;
        this.yController = yController;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.thetaController = thetaController;
        request = new SwerveRequest.ApplyFieldSpeeds();
        request.DriveRequestType = DriveRequestType.Velocity;
    }

    @Override
    public void accept(Pose2d t, SwerveSample u) {
        //This section deals with the field reletive veleocitys directly commanded of the robot

        //here the "FF" is our base setpoint, idealy, this shoud be exatly what we are commanding of our drive as thease are the chassis speeds calculated by the choreo optimizer
        double xFF = u.vx;
        double yFF = u.vy;
        double rotationFF = u.omega;

        //the feedback exists to compensate for disturmbences not modeled during trajectory generation, so basicly any real workd factors that might effect path following such as varyation from the mathamatical modle of the robot, collisions, etc
        double xFeedback = xController.calculate(t.getX(), u.x);
        double yFeedback = yController.calculate(t.getY(), u.y);
        double rotationFeedback = thetaController.calculate(t.getRotation().getRadians(),
           u.heading);

        ChassisSpeeds targetSpeeds = new ChassisSpeeds(
            xFF + xFeedback,
            yFF + yFeedback,
            rotationFF + rotationFeedback
        );

        request.Speeds = targetSpeeds;
        // request.WheelForceFeedforwardsX = u.moduleForcesX();
        // request.WheelForceFeedforwardsY = u.moduleForcesY();
        drivetrain.setControl(request);

        BreakerLog.log("BreakerSwerveChoreoController/Goal", u);
        BreakerLog.log("BreakerSwerveChoreoController/Error/X", xController.getError());
        BreakerLog.log("BreakerSwerveChoreoController/Error/Y", yController.getError());
        BreakerLog.log("BreakerSwerveChoreoController/Error/Theta", thetaController.getError());
        BreakerLog.log("BreakerSwerveChoreoController/Outputs/Feedforward", new ChassisSpeeds(xFF, yFF, rotationFF));
        BreakerLog.log("BreakerSwerveChoreoController/Outputs/Feedback", new ChassisSpeeds(xFeedback, yFeedback, rotationFeedback));
        BreakerLog.log("BreakerSwerveChoreoController/Outputs/TargetSpeeds", targetSpeeds);
    }
}
