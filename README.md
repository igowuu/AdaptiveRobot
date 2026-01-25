# AdaptiveRobot (Request-Based Architecture)

## The issue

In FRC, robots almost never do just one action at a time.

At any given moment, your robot might be:

* driving with a joystick
* running an autonomous routine
* trying to stay safe (brownout protection, emergency stop, etc.)
* having auton work in correlation with teleop

The problem is that all of these systems want to control the same hardware.

Traditionally, teams solve this with:

* conditional chains
* mode flags (`isAuto`, `isTeleop`, `isClimbing`, etc.)
* or a traditional scheduler

That works, but it tends to:

* hide where motor outputs actually come from
* make it hard to mix behaviors (for ex, auto rotation + manual driving)
* be confusing on what's being executed and what isn't

AdaptiveRobot answers this:

> **Who is allowed to control each part of a subsystem right now?**

---

## Idea

Instead of control files directly setting motor outputs, everything makes requests.

* Components *request* motion
* The subsystem itself *decides* what actually happens
* Conflicts are resolved by priority

Nothing else is allowed to directly touch a subsystems motors.

Think of it less like commands and more like raising your hand in class.
You request something by raising your hand, and hope that the teacher calls on you.
If you have the biggest priority, you'll be called on to answer; if not, you'll be ignored.
This is exactly how this whole architecture works, but rather than students raising their hands, different modes with set priorities are being chosen upon.

---

## Subsystems

A subsystem is:

* one logical chunk of the robot
* responsible for exactly one place where hardware is written
* allowed to publish intent, telemetry, and tunables

For example:

* `Drivetrain` → owns drivetrain motors
* `Shooter` → owns shooter motors
* `Vision` → owns cameras, but never motors
* `AutoAlign` (not a subsystem!) → never writes motors directly, only makes requests

So here's a rule:

> **All hardware writes must originate from exactly one subsystem per loop.
> Everything else can only publish intent.**

That rule elimanates a LOT of bugs.

## Components

A component is:

* any part of the bot that needs to be executed. So not helpers and whatnot.
* a subsystem. A subsystem *is* a specific type of component.
* a file that is tied closely to the main robot class.

Most files in the codebase should be components, let aside simple utilities.

---

## The request system (the big part)

### Why separate axes?

The tankdrive drivetrain doesn’t just drive with a chassis object - it has multiple independent axes:

* forward/backward (linear velocity)
* rotation (angular velocity)

These don’t always need to come from the same source.

Here's an example:

> Imagine a turreted robot (or even just a drivetrain).
>
> * Auto wants to control *rotation* to aim at a target
> * The driver still wants to control *linear movement.*
>
> If both try to control the same thing, then very bad things will happen.
> If they control *different axes*, then there's no conflict.

So instead of drive commands, the drivetrain exposes axis requests. More on axis requests below.

---

### Axis requests

An axis request is just:

* a value (like linear motion or rotation, in your specified units)
* an assigned priority
* a source name (for debugging / telemetry)

For example:

* Teleop joystick:

  * forward axis -> priority `TELEOP`
  * rotation axis -> priority `TELEOP`
* Auto routine:

  * rotation axis → priority `AUTO`
* Safety system:

  * forward axis → priority `SAFETY`
  * rotation axis → priority `SAFETY`

Each priority (`TELEOP`, `AUTO`, `SAFETY`) is assigned to a value by you. So, for example, `SAFETY` value > `AUTO` value > `TELEOP value` -> So safety requests are prioritized, then auto, then teleop.

Every loop, the subsystem:

1. collects all requests
2. picks the highest-priority request per axis
3. ignores the rest
4. drives the robot based on the winner for each axis

---

### Priority-based conflict resolution

Priorities are explicit and boring (but that's a really good thing!)

```python
class DrivePriority(Enum):
    SAFETY = 3
    AUTO = 2
    TELEOP = 1
```

Higher number = more important.

This means:

* Safety requests alway win over everything else
* Auto can override the driver (teleop) when needed
* Teleop can't override anything

And because this happens every loop, you can easily blend control

---

## What a subsystem actually does

A subystem is the **final authority** on movement.

It:

* owns all motors, sensors, and other hardware
* resolves axis requests every loop

Other code never sets motor outputs directly.
They just say things like:

> "I wanna to go forward at 2 m/s, priority AUTO."

The subsystem decides if that actually happens.

---

## Telemetry and debugging 

Because every request has:

* a value
* a priority
* a source

…the drivetrain can publish:

* which request won
* who lost
* what mode is currently in control

This makes debugging SO much easier than guessing which command is running.

When something goes wrong, you don’t ask:

> "Why is the robot spinning?"

You look at NetworkTables and see:

> "Rotation request from AutoAlign, priority AUTO."

---

## AdaptiveRobot

`AdaptiveRobot` is a thin wrapper around `TimedRobot` that:

* manages components
* enforces update order
* centralizes telemetry
* supports tunable constants cleanly

Each loop:

1. components publish telemetry
2. components execute
3. tunable values update *after* execution

This keeps behavior predictable.

---

## Why this isn’t Command-based

Command-based is great.
But it isn't flexible enough for *the best of the best of teams (FRC 5113)*.

This architecture:

* favors continuous intent over discrete commands
* makes mixing behaviors easy
* scales super well as robots grow

You can still structure your code cleanly, however you want. 
Everything in this architecture is optional. 
You can choose if you want some features, but don't want others.

---

## Summary

* Subsystems own hardware
* Everything else that is binded to the robot is a Component
* Controls requests
* Requests are resolved per-axis
* Priorities decide which request wins
* The subsystem is the final authority
* Mixing auto, teleop, and safety becomes predictable

---

And the best part? This works for *any type of robot, ever*. 

So no matter what your robot looks like, you're going to have a boss codebase by using this. ;3
