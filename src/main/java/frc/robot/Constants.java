package frc.robot;

public final class Constants {
  public static final class Swerve {
    public static final double halfWidth = 0.3;
    public static final double halfLength = 0.3;
  }
  public static final class RobotContainer{
    public static final double deadband = 0.05;
    public static final double MAX_SPEED = 3.0;
    public static final double MAX_ROT_SPEED = 3.0;
  }
  public static final class SwerveModule{
    public static final double wheelDiameterInches = 4.0;
    public static final double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
  }

}
