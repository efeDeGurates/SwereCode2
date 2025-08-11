package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.lang.module.Configuration;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import static frc.robot.Constants.SwerveModule.*;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;
    private final CANcoder driveEncoder;

    public SwerveModule(int driveID,int steerID,int steerEncoderID,int driveEncoderID){

        driveMotor = new TalonFX(driveID);
        steerMotor = new TalonFX(steerID);
        steerEncoder = new CANcoder(steerEncoderID);
        driveEncoder = new CANcoder(driveEncoderID);

        configureMotors();
    }

    private void configureMotors(){
        TalonFXConfiguration dirveConfiguration = new TalonFXConfiguration();
        dirveConfiguration.Slot0.kP = 0.1;
        dirveConfiguration.Slot0.kI = 0.0;
        dirveConfiguration.Slot0.kD = 0.0;
        driveMotor.getConfigurator().apply(dirveConfiguration);


        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.Slot0.kP = 1.0;
        steerConfig.Slot0.kI = 0.0;
        steerConfig.Slot0.kD = 0.0;
        steerMotor.getConfigurator().apply(steerConfig);
    }

    public void setDriveSpeed(double speed){
        
        driveMotor.setControl(new DutyCycleOut(speed)); // gücü 0-1 arasında ayarlamak için DutyCycleOut
    }

    public void setDesiredState(SwerveModuleState desiredstate){
        Rotation2d currentangle = Rotation2d.fromDegrees(getSteerAngle());

        SwerveModuleState optimixedstate = SwerveModuleState.optimize(desiredstate, currentangle);

        setDriveSpeed(optimixedstate.speedMetersPerSecond);

        setDriveAngle(optimixedstate.angle.getDegrees());
    }

    public void setDriveAngle(double angleDegrees){

        double targetPositionRotations = angleDegrees / 360.0;

        steerMotor.setControl(new PositionVoltage(targetPositionRotations));

    }

    public double getSteerAngle(){
        return steerEncoder.getPosition().getValueAsDouble() * 360.0;
    }
    public double getDriveDistanceMeters() {
        double rotations = driveEncoder.getPosition().getValueAsDouble();
        double wheelCircumferenceMeters = wheelCircumferenceInches * 0.0254;
        double distanceMeters = rotations * wheelCircumferenceMeters;
        return distanceMeters;
    }
    public SwerveModulePosition getPosition() {
    double distanceMeters = getDriveDistanceMeters();
    Rotation2d angle = Rotation2d.fromDegrees(getSteerAngle());
    return new SwerveModulePosition(distanceMeters, angle);
}
}

