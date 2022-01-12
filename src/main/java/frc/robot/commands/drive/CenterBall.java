package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.sensors.IntakeCam;
import java.util.function.DoubleSupplier;

/** Drives forward a certain percent while keeping the drivetrain centered on the vision target. */
public class CenterBall extends CommandBase {

  // initialize class variables for subsystems and input.
  private final Swerve drivetrain = Swerve.getInstance();
  private final IntakeCam intakecam = IntakeCam.getInstance();
  private final DoubleSupplier xDrive;
  private final DoubleSupplier yDrive;
  private boolean cont;

  // instantiate a PID controller to control the rotation of the robot.
  private final double kP = 0.008;
  private final double kD = 0.0;
  private final PIDController pid = new PIDController(kP, 0, kD);

  /**
   * Constructs a new CenterBay command to drive forward a certain percent while keeping the
   * drivetrain centered on the vision target.
   *
   * @param drivetrain - Drivetrain subsystem to require.
   * @param intakecam - Camera subsystem to use.
   * @param percentForward - Supplier of percent to drive forward [-1.0, 1.0].
   */
  public CenterBall(DoubleSupplier xDrive, DoubleSupplier yDrive, boolean cont) {
    // Require the drivetrain for this command so that it is the only one controlling the drivetrain
    // when it is running.
    addRequirements(drivetrain);
    this.xDrive = xDrive;
    this.yDrive = yDrive;
    this.cont = cont;
  }

  public CenterBall(boolean cont) {
    // Require the drivetrain for this command so that it is the only one controlling the drivetrain
    // when it is running.
    addRequirements(drivetrain);
    xDrive = () -> 0.0;
    yDrive = () -> 0.0;
    this.cont = cont;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Set the rotation of the robot equal to the PID Controller output (calculated from X angle of
    // the closest ball).
    double zRotation = pid.calculate(-intakecam.getCloseX(0.0), 0);
    drivetrain.drive(xDrive.getAsDouble(), yDrive.getAsDouble(), zRotation, false);
  }

  @Override
  public boolean isFinished() {
    return !cont;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setChassisSpeeds(new ChassisSpeeds());
  }
}
