package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class VisionSubsystem implements Subsystem {
 
// Components (e.g., motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  // Example private motor controller:
  // private final PWMVictorSPX m_motor = new PWMVictorSPX(4);

  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem() {
    // Constructor for the subsystem, used for initial setup and instantiation of components.
  }

  /**
   * Called periodically whenever the CommandScheduler runs.
   * This is useful for "background" actions or logging data to the dashboard.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
  }

  // Public methods to control the subsystem's components (e.g., setting motor speeds, reading sensor data)
  // Example public method:
  // public void runMotor(double speed) {
  //   m_motor.set(speed);
  // }
}


