// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. 
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve_drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
private final Joystick driverL;
  private final Joystick driverR;

  private final Swerve_drive driveSwerve;
  // private final BooleanSupplier robotCentricSupply;

  private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3); // limit "acceleration" in translation rotation and strafe
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3);

  private double strafeVal;
  private double translationVal;
  private double rotationVal;

  private boolean isAuto;

  /** Creates a new Drive. */
  public Drive(Swerve_drive driveSwerve, Joystick driverL, Joystick driverR, double strafeVal, double translationVal, double rotationVal, boolean isAuto) {
    this.driveSwerve = driveSwerve;
    this.driverL = driverL;
    this.driverR = driverR;
    this.strafeVal = strafeVal;
    this.translationVal = translationVal;
    this.rotationVal = rotationVal;
    this.isAuto = isAuto;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double strafe_value;
    double translation_value;
    double rotation_value;

if (isAuto){
  strafe_value = strafeLimiter.calculate(MathUtil.applyDeadband(strafeVal, 0.1));
  translation_value = translationLimiter.calculate(MathUtil.applyDeadband(translationVal, 0.1)); //apply deadband to ignore small input fluctuations
  rotation_value = rotationLimiter.calculate(MathUtil.applyDeadband(rotationVal, 0.1));

}
    else{ //if youre reading this you are a naughty naughty boy 🗡️

      strafe_value = strafeLimiter.calculate(MathUtil.applyDeadband(this.driverL.getRawAxis(0), 0.1));
      translation_value = translationLimiter.calculate(MathUtil.applyDeadband(this.driverL.getRawAxis(1), 0.1)); //applies deadbandd
      rotation_value = rotationLimiter.calculate(MathUtil.applyDeadband(this.driverR.getRawAxis(0), 0.1)); 
    }
    driveSwerve.Drive(new Translation2d(translation_value, strafe_value).times(Constants.maxSpeed), rotation_value * Constants.maxAngularVelocity, true, false);
      SmartDashboard.putNumber("translation value", translation_value);
      SmartDashboard.putNumber("strafe value", strafe_value);             // puts values on smart dshboard
      SmartDashboard.putNumber("rotation value", rotation_value);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
