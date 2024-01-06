package frc.robot.subsystems.shooter;

public class ShooterSetpoints {
  public double hoodAngleRad = 0.0;
  public double turretAngleRad = 0.0;
  public double rpm = 0.0;

  public ShooterSetpoints(double hoodAngleRad, double turretAngleRad, double rpm) {
    this.hoodAngleRad = hoodAngleRad;
    this.turretAngleRad = turretAngleRad;
    this.rpm = rpm;
  }

  public ShooterSetpoints() {
    this(0, 0, 0);
  }

  public static final ShooterSetpoints DEFAULT =
      new ShooterSetpoints(Math.toRadians(45.0), Math.toRadians(0.0), 0.0);
}
