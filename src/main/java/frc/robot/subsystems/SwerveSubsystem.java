package frc.robot.subsystems;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Swerve.*;

import com.ctre.phoenix6.hardware.Pigeon2;

public class SwerveSubsystem extends SubsystemBase{

    private final SwerveModule frontrightModule;
    private final SwerveModule frontleftModule;
    private final SwerveModule backrightModule;
    private final SwerveModule backlefModule;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final Pigeon2 gyro;

    private final SwerveDrivePoseEstimator poseEstimator;

    public SwerveSubsystem(){

        frontrightModule = new SwerveModule(1, 2, 3, 4);
        frontleftModule = new SwerveModule(5, 6, 7, 8);
        backrightModule = new SwerveModule(9, 10, 11, 12);
        backlefModule = new SwerveModule(13, 14, 15, 16);

        kinematics = new SwerveDriveKinematics(
            new Translation2d(halfLength, halfWidth),
            new Translation2d(halfLength, -halfWidth),
            new Translation2d(-halfLength, halfWidth),
            new Translation2d(-halfLength, -halfWidth)
        );

        gyro = new Pigeon2(13);
        gyro.reset();

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getHead(), getPositions(), new Pose2d());

        odometry = new SwerveDriveOdometry(
            kinematics,
            getHead(),
            getPositions()
        );
    }
    public void updateOdometry(){
        odometry.update(getHead(), getPositions());
    }

    public void drive(double xspeed,double yspeed,double rotspeed){
        Rotation2d robotAngle = getHead();
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xspeed, yspeed, rotspeed, robotAngle
        );
        
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

        frontleftModule.setDesiredState(states[0]);
        frontrightModule.setDesiredState(states[1]);
        backlefModule.setDesiredState(states[2]);
        backrightModule.setDesiredState(states[3]);
    }

    
    public Rotation2d getHead(){
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetGyro(){
        gyro.setYaw(0);
    }

    public SwerveModulePosition[] getPositions(){
        return new SwerveModulePosition[]{
            frontleftModule.getPosition(),
            frontrightModule.getPosition(),
            backlefModule.getPosition(),
            backrightModule.getPosition()
        };
    }

    // offset
    public void saveAllModuleOffset(){
        frontleftModule.saveOffset();
        frontrightModule.saveOffset();
        backlefModule.saveOffset();
        backrightModule.saveOffset();
    }
    
    public void updateApriltagOdometry(){
        double[] pos = NetworkTableInstance.getDefault()
        .getTable("apriltag")
        .getEntry("tag_3/pos")
        .getDoubleArray(new double[0]);

        if(pos.length==3){
            double x = pos[0];
            double y = pos[1];
            double heading = pos[2];

            Pose2d visionPose = new Pose2d(x,y,new Rotation2d(heading));
            poseEstimator.addVisionMeasurement(
            visionPose,
            Timer.getFPGATimestamp()
        );
        }
    }
    public SwerveDrivePoseEstimator getposeEstimator(){
        return poseEstimator;
    }
}
