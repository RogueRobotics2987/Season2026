// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

public class UtilitiesSubsystem extends SubsystemBase {
    private CommandXboxController _joystick;
    private CommandXboxController _auxJoystick;
    private boolean redInactiveFirst = false;
    private boolean shift1Active = false;
    private double matchTime;

    private Optional<Alliance> alliance = Optional.empty();

    private int counter = 0; // for troubleshooting
    private String gameData;

    /** Creates a new UtilitiesSubsystem. */
    public UtilitiesSubsystem(CommandXboxController joystick, CommandXboxController auxJoystick) { 
        _joystick = joystick;
        _auxJoystick = auxJoystick;
    }

    public boolean isHubActive(double matchTime) {
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if(alliance == null || alliance.isEmpty()){
            alliance = DriverStation.getAlliance();
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
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        //TODO Verify this assumtion is correct CKuoppala
        if (gameData == null || gameData.isEmpty()) {
            gameData = DriverStation.getGameSpecificMessage();
            return true;
        } else {
            switch (gameData.charAt(0)) {
                case 'R' -> redInactiveFirst = true;
                case 'B' -> redInactiveFirst = false;
                default -> {
                    // If we have invalid game data, assume hub is active.
                    return true;
                }
            }
        }

        

        // Shift was is active for blue if red won auto, or red if blue won auto.
         shift1Active = switch (alliance.get()) {
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
        matchTime = DriverStation.getMatchTime();
        SmartDashboard.putBoolean("Hub Enabled", isHubActive(matchTime));

    } // end of periodic

    // command to rumble according to specific remaining shift time, 10 sec (0), and 5 sec (1) remaining
    public Command rumbleController(int mode) {
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

}  // closes subsystem