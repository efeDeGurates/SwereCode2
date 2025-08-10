package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Swerve.*;

public class SwerveSubsystem extends SubsystemBase{

    private final SwerveModule frontrightModule;
    private final SwerveModule frontleftModule;
    private final SwerveModule backrightModule;
    private final SwerveModule backlefModule;

    private final SwerveDriveKinematics kinematics;

    public SwerveSubsystem(){
        frontrightModule = new SwerveModule(1,2,3);
        frontleftModule = new SwerveModule(4,5,6);
        backrightModule = new SwerveModule(7,8,9);
        backlefModule = new SwerveModule(10,11,12);

        kinematics = new SwerveDriveKinematics(
            new Translation2d(halfLength,  halfWidth),
            new Translation2d(halfLength, -halfWidth),
            new Translation2d(-halfLength, halfWidth),
            new Translation2d(-halfLength, -halfWidth) 
        );
    }

    public void drive(double xspeed,double yspeed,double rotspeed){
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
            new edu.wpi.first.math.kinematics.ChassisSpeeds(xspeed,yspeed,rotspeed)
        );

        frontleftModule.setDriveSpeed(states[0].speedMetersPerSecond);
        frontleftModule.setDriveAngle(states[0].angle.getDegrees());

        frontrightModule.setDriveSpeed(states[1].speedMetersPerSecond);
        frontrightModule.setDriveAngle(states[1].angle.getDegrees());

        backlefModule.setDriveSpeed(states[2].speedMetersPerSecond);
        backrightModule.setDriveAngle(states[2].angle.getDegrees());

        backlefModule.setDriveSpeed(states[3].speedMetersPerSecond);
        backlefModule.setDriveAngle(states[3].angle.getDegrees());
    }
    
}
