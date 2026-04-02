// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//things to do: customize controller rumbles (for 10 and 5 sec), set rumble calls at appropriate times
//Also: remove unnecessary comments when done
package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class UtilitiesSubsystem extends SubsystemBase {
  /** Creates a new UtilitiesSubsystem. */
private CommandXboxController _joystick;
private CommandXboxController _auxJoystick;
//variables for the match times where we want the controller to rumble
private int shift1_05 = 135;
private int shift2_10 = 115;
private int shift2_05 = 110;
private int shift3_10 = 90;
private int shift3_05 = 85;
private int shift4_10 = 65;
private int shift4_05 = 60;

private int counter = 0; // for troubleshooting
// flags for the rumbles so that they dont rumble twice (may not use now)
// public boolean didShift1_05 = false;
// public boolean didShift2_10 = false;
// public boolean didShift2_05 = false;
// public boolean didShift3_10 = false;
// public boolean didShift3_05 = false;
// public boolean didShift4_10 = false;
// public boolean didShift4_05 = false;


  public UtilitiesSubsystem(CommandXboxController joystick, CommandXboxController auxJoystick) { 
    _joystick = joystick;
    _auxJoystick = auxJoystick;
  }

  public boolean isHubActive(double matchTime) {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      // If we have no alliance, we cannot be enabled, therefore no hub.
      if (alliance.isEmpty()) {
          return false;
      }
      // Hub is always enabled in autonomous.
      if (DriverStation.isAutonomousEnabled()) {
          return true;
      }
      // At this point, if we're not teleop enabled, there is no hub.
      if (!DriverStation.isTeleopEnabled()) {
          return false;
      }
      
      // We're teleop enabled, compute.
      String gameData = DriverStation.getGameSpecificMessage();
      // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
      //TODO Verify this assumtion is correct CKuoppala
      if (gameData.isEmpty()) {
          return true;
      }
      boolean redInactiveFirst = false;
      switch (gameData.charAt(0)) {
          case 'R' -> redInactiveFirst = true;
          case 'B' -> redInactiveFirst = false;
          default -> {
              // If we have invalid game data, assume hub is active.
              return true;
          }
      }

      // Shift was is active for blue if red won auto, or red if blue won auto.
      boolean shift1Active = switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
      };

      if (matchTime > 130) {
          // Transition shift, hub is active.
          return true;
      } else if (matchTime > 105) {
          // Shift 1
          return shift1Active;
      } else if (matchTime > 80) {
          // Shift 2
          return !shift1Active;
      } else if (matchTime > 55) {
          // Shift 3
          return shift1Active;
      } else if (matchTime > 30) {
          // Shift 4
          return !shift1Active;
      } else {
          // End game, hub always active.
          return true;
      }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double matchTime = DriverStation.getMatchTime();
    SmartDashboard.putBoolean("Hub Enabled", isHubActive(matchTime));
   // countSec();
   //SmartDashboard.putNumber("Rumble Status", matchTime);
   // for troubleshooting: SmartDashboard.putNumber("Rumble Status", 140 - counter);
    //rumble logic; checks for matchtime to go through rumble once (0) or twice (1)

    new Trigger(() -> matchTime <= shift1_05)
  .onTrue(rumbleController(1));

new Trigger(() -> matchTime <= shift2_10)
.onTrue(rumbleController(0));

new Trigger(() -> matchTime <= shift2_05)
.onTrue(rumbleController(1));

new Trigger(() -> matchTime <= shift3_10)
.onTrue(rumbleController(0));

new Trigger(() -> matchTime <= shift3_05)
.onTrue(rumbleController(1));

new Trigger(() -> matchTime <= shift4_10)
.onTrue(rumbleController(0));

new Trigger(() -> matchTime <= shift4_05)
.onTrue(rumbleController(1));

  } // end of periodic

  // command to rumble according to specific remaining shift time, 10 sec (0), and 5 sec (1) remaining
private Command rumbleController(int mode){

 if (mode == 0) {
        return singleRumble(1.0);
    }

    else if (mode == 1) {
        return Commands.sequence(
            singleRumble(0.5),
            Commands.waitSeconds(0.2),
            singleRumble(0.5)
        );
    }

    return Commands.none();

} // closes runbleController
// function for a single rumble, can be set to 1 sec or half (0.5) sec
private Command singleRumble(double length) {

return Commands.startEnd(
  () -> {
    _joystick.setRumble(RumbleType.kBothRumble, 1.0);
     _auxJoystick.setRumble(RumbleType.kBothRumble, 1.0);

 },

  () -> {
    _joystick.setRumble(RumbleType.kBothRumble, 0.0);
    _auxJoystick.setRumble(RumbleType.kBothRumble, 0.0);
 }
) .withTimeout(length);
} // closes singleRumble

// private void countSec() {
//     counter+= 1;
// }

}  // closes subsystem

      