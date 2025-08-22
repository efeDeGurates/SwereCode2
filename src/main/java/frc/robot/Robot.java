package frc.robot;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.ObjectInputFilter.Config;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private NetworkTable apriltagTable;

  public Robot() {
    m_robotContainer = new RobotContainer();
    apriltagTable = NetworkTableInstance.getDefault().getTable("apriltags");
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    swerveSubsystem.getposeEstimator().update(
    swerveSubsystem.getHead(),
    swerveSubsystem.getPositions()
);
    swerveSubsystem.updateApriltagOdometry();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
