# Commands: How To Document

## What is Command-Based Programming?

In FRC, command-based programming is a way to organize your robot code so that every action the robot takes is a **Command**, and every piece of hardware is managed by a **Subsystem**. The **Scheduler** is the traffic cop — it makes sure only one command controls a subsystem at a time, preventing hardware conflicts.

Think of it like a restaurant:
- **Subsystems** are the kitchen stations (grill, fryer, prep table).
- **Commands** are the orders ("make a burger", "fry some fries").
- **The Scheduler** is the head chef who makes sure two cooks aren't fighting over the same grill.

Before we dive into commands, you need to understand one critical concept: **requirements**.

---

## Requirements: Why They Matter

When a command "requires" a subsystem, it tells the Scheduler: "I need exclusive access to this hardware." If two commands both require the same subsystem, the Scheduler will cancel the old one and let the new one take over.

If you forget to declare requirements, the Scheduler has no idea your command uses that subsystem. This means two commands could try to control the same motor at the same time — and your robot does something unpredictable.

**Rule of thumb:** If your command touches a subsystem's hardware, it must require that subsystem.

---

## Two Ways to Create Commands

There are two main styles you'll see in FRC code. Understanding the difference is important.

### Style 1: Generic Lambda — `Commands.runOnce(() -> { ... })`

```java
joystick.a().onTrue(Commands.runOnce(() -> { brakeEnabled = !brakeEnabled; }));
```

- Creates a generic `InstantCommand` that does **not** automatically know which subsystem it uses.
- The Scheduler won't manage conflicts because no subsystem requirement is declared.
- **Use for:** Simple internal state changes (flipping a boolean, logging) that don't touch hardware.

### Style 2: Subsystem Factory — `m_subsystem.runOnce(m_subsystem::method)`

```java
AuxJoystick.rightTrigger().onTrue(m_IndexSubsystem.runOnce(m_IndexSubsystem::start));
```

- This is a **subsystem factory method**. It automatically adds the subsystem as a requirement.
- The Scheduler knows this command owns that subsystem and will handle conflicts.
- **Use for:** Any action that interacts with subsystem hardware or core logic. This is the preferred approach.

### Quick Comparison

| | `Commands.runOnce(() -> ...)` | `m_subsystem.runOnce(...)` |
|---|---|---|
| Subsystem requirement | None (manual) | Automatic |
| Scheduler conflict handling | No | Yes |
| Best for | Internal state, logging | Hardware control |

---

## The Factory Pattern

The "factory pattern" is the recommended way to write commands in modern FRC. Instead of writing command logic inside your button bindings in RobotContainer, you define **factory methods** inside your Subsystem class that return `Command` objects.

### Why Use Factories?

1. **Encapsulation:** RobotContainer doesn't need to know *how* a subsystem works — only *what* it can do.
2. **Reusability:** You can call `m_index.getStartCommand()` in multiple places (buttons, autonomous routines, command groups) without rewriting logic.
3. **Readability:** Button bindings become a clean list of intentions instead of a mess of lambdas.
4. **Safety:** Stop/cleanup logic is defined once inside the factory. You can't accidentally forget it when binding a new button.

---

## Two Flavors of the Factory Pattern

### 1. Inline Factory

You use the built-in factory methods (`runOnce`, `run`, `startEnd`, etc.) provided by the `Subsystem` class, directly in RobotContainer.

```java
// RobotContainer.java
AuxJoystick.rightTrigger().onTrue(m_IndexSubsystem.runOnce(m_IndexSubsystem::start));
```

- The **Subsystem** provides a simple `void` method.
- **RobotContainer** decides *how* that method is run (once, continuously, etc.).
- Good for quick one-offs, but the "how" logic lives in RobotContainer.

### 2. Custom Factory (Recommended)

You create your own method inside the Subsystem that returns a fully configured `Command`.

```java
// IndexSubsystem.java
public void start() {
    m_motor.set(1.0);  // The "action" — simple hardware control
}

public Command getStartCommand() {
    return this.runOnce(this::start);  // The "factory" — returns a Command
}
```

```java
// RobotContainer.java
AuxJoystick.rightTrigger().onTrue(m_IndexSubsystem.getStartCommand());
```

- The **Subsystem** decides exactly how the command behaves.
- **RobotContainer** just asks for the plan and hands it to the button.
- This is the cleanest separation of concerns.

### The Hybrid Approach (What Most Top Teams Do)

1. Keep hardware methods as simple `void` functions (e.g., `setSpeed(double speed)`).
2. Create **Custom Factories** for common tasks (e.g., `getStopCommand()`).
3. Use **Inline Factories** in RobotContainer for one-off things that don't need a dedicated method.

---

## Command Composition: Built-in Factory Methods

WPILib provides several composition methods to handle start/run/stop logic. Here's when to use each one.


### `runOnce` — Do Something Once

Runs the action a single time and immediately finishes. The motor (or whatever you set) stays in that state until something else changes it.

```java
// IndexSubsystem.java
public Command getStartCommand() {
    return this.runOnce(() -> m_motor.set(1.0));
}
```

**Use for:** Instant state changes — starting a motor, flipping a boolean, toggling a solenoid.

### `startEnd` — Do Something, Then Clean Up

Takes two functions: one for when the command starts, and one for when it ends (whether it finishes naturally or is interrupted). This is the most common way to handle "while held" logic.

```java
// IndexSubsystem.java
public Command getRunIndexerCommand() {
    return this.startEnd(
        () -> m_motor.set(1.0),   // What to do at START
        () -> m_motor.set(0.0)    // What to do at END
    );
}
```

**Use for:** Actions that must be cleaned up — stopping a motor when a button is released, closing a solenoid.

### `runEnd` — Continuously Update, Then Clean Up

Like `startEnd`, but the first function runs **every 20ms** (every scheduler cycle) instead of just once. Use this when you need to constantly update something while the command is active.

```java
// IndexSubsystem.java
public Command getSafeRunCommand() {
    return this.runEnd(
        () -> m_motor.set(m_speedPot.get()),  // Runs EVERY 20ms
        () -> m_motor.set(0)                   // Stop at the end
    );
}
```

**Use for:** PID loops, sensor-based adjustments, or anything that needs continuous updates.

### `run` — Continuously Update (No Built-in End Logic)

Runs the action every 20ms with no automatic end behavior. You'll typically chain `.finallyDo()` or use it with `.whileTrue()` on a trigger.

```java
// IndexSubsystem.java
public Command getManualCommand() {
    return this.run(() -> m_motor.set(m_speedPot.get()));
}
```

**Use for:** Default commands or situations where you handle the end logic separately.

### Adding Duration with `withTimeout`

You can chain a timeout onto any command. This tells the Scheduler: "Run this plan, but force it to finish after X seconds."

```java
// IndexSubsystem.java
public Command getTimedIndexCommand() {
    return this.runEnd(
        () -> m_motor.set(1),
        () -> m_motor.set(0)
    ).withTimeout(2.0);  // Automatically ends after 2 seconds
}
```

**Use for:** Autonomous-style actions where something should happen for a specific duration.

### Quick Reference Table

| Method | Runs at Start | Runs Every 20ms | Runs at End | Best For |
|---|---|---|---|---|
| `runOnce` | Yes | No | No | Instant state changes |
| `startEnd` | Yes | No | Yes | Toggle on/off with cleanup |
| `runEnd` | No | Yes | Yes | Continuous updates with cleanup |
| `run` | No | Yes | No | Default commands, manual control |

---

## Method References vs. Lambdas

You'll see two syntax styles for passing functions to commands. They do the same thing — the choice depends on what you need.

### `::` Method Reference

Use when you're calling exactly one existing method with **no arguments**. Cleanest looking option.

```java
m_IndexSubsystem.runOnce(m_IndexSubsystem::start)
```

### `() ->` Lambda

Use when you need to:

1. **Pass arguments:**
   ```java
   // Works
   () -> turretSubsystem.setSpeed(0.5)
   // Does NOT work — method references can't take arguments
   // turretSubsystem::setSpeed(0.5)
   ```

2. **Do multiple things:**
   ```java
   () -> {
       turretSubsystem.DisableShooter();
       statusLight.set(Red);
   }
   ```

3. **Use a value that changes over time (Supplier):**
   ```java
   () -> m_motor.set(m_controller.getRightTriggerAxis())
   ```

**Rule of thumb:** If it's one method with no arguments, use `::`. Otherwise, use `() ->`.

---

## Parameterized Factories: Passing Variables to Commands

This is where the factory pattern gets really powerful. You can write a single factory method that accepts parameters, letting RobotContainer configure behavior without knowing hardware details.

### Fixed Parameters (Presets)

```java
// ShooterSubsystem.java

// The "action" — simple hardware control
public void setShooterSpeed(double speed) {
    m_motor.set(speed);
}

// The "factory" — builds a command based on input
public Command getShootCommand(double speed) {
    return this.runEnd(
        () -> setShooterSpeed(speed),   // Start: set the speed
        () -> setShooterSpeed(0)        // End: always stop
    );
}
```

```java
// RobotContainer.java
m_driverController.a().whileTrue(m_shooter.getShootCommand(0.4));  // Low goal
m_driverController.y().whileTrue(m_shooter.getShootCommand(1.0));  // High goal
```

The stop logic is defined **once** inside the factory. Both buttons automatically get it.

### Dynamic Parameters (Suppliers)

What if you want the speed to come from a joystick axis that changes constantly? Pass a `DoubleSupplier` instead of a fixed `double`.

```java
// ShooterSubsystem.java
public Command getManualShootCommand(DoubleSupplier speedSupplier) {
    return this.run(() -> setShooterSpeed(speedSupplier.getAsDouble()));
}
```

```java
// RobotContainer.java
m_shooter.setDefaultCommand(
    m_shooter.getManualShootCommand(() -> m_operatorController.getRightTriggerAxis())
);
```

**When to use which:**
- **Fixed value (like `0.4`):** The value is a preset that never changes.
- **Supplier (like `() -> axis`):** The value changes in real-time (joystick input, sensor reading).

---

## Multi-Subsystem Commands: Command Composition

A subsystem factory usually only controls its own hardware. When you need to coordinate multiple subsystems, you use **Command Composition** to build "Super Commands."

This is typically done in RobotContainer because that's where all the subsystems meet.


### Sequential: `.andThen()` — Do A, Then B

Commands run one after another. The next command starts only when the previous one finishes.

```java
// RobotContainer.java — Aim the shooter, then run the indexer for 2 seconds
m_driverController.a().onTrue(
    m_shooter.aimCommand()
        .andThen(
            m_index.getRunCommand()
                .withTimeout(2.0)
        )
);
```

### Parallel: `.alongWith()` — Do A and B at the Same Time

Commands run simultaneously. The group finishes when **all** commands finish.

```java
// RobotContainer.java — Spin intake while arm moves down
m_driverController.rightBumper().whileTrue(
    m_arm.moveDownCommand()
        .alongWith(m_intake.spinCommand())
);
```

### Parallel Race: `.raceWith()` — First One Done Wins

Commands run simultaneously. The group finishes as soon as **any one** command finishes, and all others are cancelled.

```java
// Intake until the sensor detects a game piece
m_intake.spinCommand()
    .raceWith(Commands.waitUntil(m_intake::hasGamePiece))
```

**Use for:** "Do X until Y happens" patterns — like intaking until a sensor triggers.

### Parallel Deadline: `.deadlineFor()` — One Command is the Boss

One command is the "deadline." When it finishes, everything else is cancelled.

```java
// Arm is the boss — when it reaches position, intake stops
m_arm.moveDownCommand()
    .deadlineFor(m_intake.spinCommand())
```

**Use for:** When one action's completion should dictate when everything stops.

### Composition Quick Reference

| Method | Finishes When | Cancels Others? | Best For |
|---|---|---|---|
| `.andThen()` | Sequential — A finishes, then B runs | No | Step-by-step sequences |
| `.alongWith()` | All commands finish | No | Simultaneous actions |
| `.raceWith()` | Any one command finishes | Yes, all others | "Do until" patterns |
| `.deadlineFor()` | The "boss" command finishes | Yes, all others | One action controls timing |

### Why Composition is Powerful

1. **Requirement Safety:** The Scheduler knows the composed command requires all involved subsystems. If another button tries to use one of those subsystems, the Scheduler handles the conflict.
2. **No Fighting:** You don't need complex `if/else` logic or manual timers.
3. **Readability:** Your code reads like a timeline — "do this, then do that, while also doing this."

### Conditional Composition with `.until()`

You can stop a chain based on a condition:

```java
// Run the indexer until a ball is loaded
m_index.getRunCommand().until(m_index::isBallLoaded)
```

---

## Putting It All Together: Multi-Subsystem Presets

Here's a complete example combining parameterized factories with command composition.

### Step 1: Subsystem Factories

```java
// ArmSubsystem.java
public Command getSetPositionCommand(double degrees) {
    return this.run(() -> m_pid.setSetpoint(degrees));
}

public boolean atSetpoint() {
    return m_pid.atSetpoint();
}
```

```java
// ShooterSubsystem.java
public Command getSpinUpCommand(double rpm) {
    return this.run(() -> m_motor.setReference(rpm));
}
```

### Step 2: "Super Factory" in RobotContainer

```java
// RobotContainer.java
public Command getScoreHighCommand() {
    return m_arm.getSetPositionCommand(90.0)
        .alongWith(m_shooter.getSpinUpCommand(4500.0))
        .withName("Score High");  // Optional: names it for the Dashboard
}
```

### Step 3: Bind It

```java
// RobotContainer.java
m_driverController.y().whileTrue(getScoreHighCommand());
```

### Step 4: Make It Safer with Sensor Checks

What if you don't want the shooter to fire until the arm is actually in position?

```java
// RobotContainer.java
public Command getSafeScoreCommand() {
    return m_arm.getSetPositionCommand(90.0)
        .alongWith(
            Commands.waitUntil(m_arm::atSetpoint)
                .andThen(m_shooter.getSpinUpCommand(4500.0))
        );
}
```

This says: "Move the arm to 90°. At the same time, wait until the arm is at its setpoint, *then* spin up the shooter."

### Why This Pattern Wins

1. **Encapsulation:** The Subsystem knows how to move a motor. The Command knows the sequence. RobotContainer knows when to trigger it.
2. **Autonomous Reusability:** You can use `getScoreHighCommand()` in both Teleop button bindings and Autonomous routines without rewriting anything.
3. **Single Source of Truth:** Change the arm angle or shooter RPM in one place, and every button and auto routine that uses it gets the update.

---

## Where Should Commands Live?

| Scenario | Where to Put It |
|---|---|
| Single-subsystem action | Factory method inside the Subsystem |
| Simple multi-subsystem chain | Directly in RobotContainer button bindings |
| Complex or reused multi-subsystem logic | A helper method in RobotContainer that returns a `Command` |
| Very complex autonomous sequences | A dedicated Command class file |

---

## Common Mistakes to Avoid

1. **Forgetting `addRequirements()`:** If you write a standalone Command class (not using factory methods), you must call `addRequirements(subsystem)` in the constructor. Without it, the Scheduler can't manage conflicts.

   ```java
   public IntakeCommand(IntakeSubsystem intakeSubsystem) {
       m_IntakeSubsystem = intakeSubsystem;
       addRequirements(intakeSubsystem);  // Don't forget this!
   }
   ```

2. **Using `Commands.runOnce()` for hardware control:** This creates a command with no subsystem requirement. Use `m_subsystem.runOnce()` instead so the Scheduler knows what hardware is involved.

3. **Not handling the "end" state:** If a command starts a motor, make sure something stops it. Use `startEnd` or `runEnd` factories, or add `.finallyDo()` to ensure cleanup happens even if the command is interrupted.

4. **Putting too much logic in RobotContainer:** If you find yourself writing complex lambdas in button bindings, that's a sign the logic should be a factory method inside the subsystem.

---

## Summary: The Decision Flowchart

```
Do I need a command?
├── Is it a simple state change (no hardware)? → Commands.runOnce(() -> ...)
├── Does it touch ONE subsystem?
│   ├── Is it a one-off? → m_subsystem.runOnce(m_subsystem::method)  (Inline Factory)
│   └── Will I reuse it? → m_subsystem.getMyCommand()  (Custom Factory)
└── Does it touch MULTIPLE subsystems?
    ├── Simple chain? → Compose in RobotContainer with .andThen() / .alongWith()
    └── Complex/reused? → Helper method in RobotContainer or dedicated Command class
```
