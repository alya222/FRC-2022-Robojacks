// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;


public class Shooter extends SubsystemBase {

  // add a piston for shooter
  private Solenoid shooterPiston = new Solenoid(compressorModule, shooterPort);

  /** Creates a new Shooter. */
  public Shooter() {
  }

  public void setShoot(boolean shoot) {
      shooterPiston.set(shoot);
  }

  public boolean shootPistonExtended() {
    return shooterPiston.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
