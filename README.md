# AdaptiveRobot (Request-Based Architecture)

![Python](https://img.shields.io/badge/python-3.11-blue)
![RobotPy](https://img.shields.io/badge/RobotPy-2026-blue)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## The issue

In typical FRC robot code:
- It's hard to coordinate multiple subsystems in teleop + auto.
- Each subsystem often has lots of boilerplate: PID setup, tunables, telemetry, deadbands, etc.
- Limited dynamic control: adding new input sources, changing priorities, or testing in sim often requires changing multiple files.
- Limited structural mutability: widely-used architectures generally enforce a strict structure of the codebase.
- Safety is not guaranteed: teleop commands can accidentally override autonomous actions or move motors past limits.
- Telemetry and tunable values are often a pain to add manually.

## What is request-based?

- Each method that typically requests motion within a subsystem becomes a setter rather than directly commanding hardware.
- Hardware is commanded by a single method called within the execute() method each iteration. execute() is called automatically by the Adaptive scheduler.
- Requests from the setter methods may be rejected or accepted based on priority.

Here's how:

- You define priorities at the top of the file. Generally, safety requests should have priority (and therefore a greater value) over all other controls.
- Each request (setter) method assigns one of these modes to their request.

For example a request could be:
`"Drive forward with a voltage of 9 volts, mode=TELEOP"`

So what happens if multiple requests are made in an iteration?

Only one request will be accepted, and the rest will gracefully expire or be explicitly cleared.

*Here's what an example of a priority enum would look like:*
```
# Higher enum value = higher priority to override other requests
# In this case, SAFETY requests can override AUTO requests. SAFETY and AUTO requests can override TELEOP requests.
class DrivePriority(Enum):
    SAFETY = 3
    AUTO = 2
    TELEOP = 1
```

And let's say that these two requests were made:

`"Drive forward with a voltage of 10 volts, mode=AUTO"`
`"STOP all drivetrain movement, mode=SAFETY"`

The request order would not matter, as long as they were both made this iteration. 
The stop request from the SAFETY mode would be executed rather than the AUTO request (or any other requests made that iteration) because it has a larger priority.

Requests are made per-axis. This means that for each type of motion of your subsystem (omega, vertical motion), requests will be handled for each of them individually within execute().

### But what if multiple requests of the same priority are requested in an iteration?

- The most recent request will always win when resolving the winning request.

So think of it like this:
In a class of students, the teacher asks a question, and multiple students raise their hand. The student with the most priority in the teachers eyes will be picked, and all the rest of the students will be ignored. In a nutshell, the system acts as the teacher, picking the request with the most priority over all others.

### Additional safety

All requests should be cleared at the end of execute() per axis by the user. This will be built into the scheduler on the next update.

In the case that this fails to happen, requests have a set (and configurable) timeout, in which they will expire if they are not fulfilled within that duration. 
This further prevents faulty or unintended requests from persisting if brownout or any other discrepancies occur.

## Why do all of this?

*Safety, debuggability, and flexability.*
- Per-axis requests enable different movements to be commanded separately. So, in other words, you'll have the ability to program a cool robot turret where the driver controls linear motion and an autonomous sequence controls the robot angle to face towards a target. Easily.
- The same hardware will only be able to be commanded once per loop, no matter what, solving many conflicting behavior and dramatically increasing bot safety.
- execute() is the single source of truth within your codebase. If something goes wrong, it's there. This makes debugging much easier for your team.

## Other features:

Though mentioned briefly above, clarification here is necessary:

- All components inherit from AdaptiveComponent.
- Each component has an optional publish_telemetry() implementation. publish_telemetry() runs right before execute(), making it easier (and encouraging users) to use telemetry per-component.
- Each component has a required execute() method, which runs automatically every iteration to reduce boilerplate and prevent hidden dependencies or misuse. **execute() is the sole source of truth within all components**.
- Each component has built-in useful methods, including (but not limited to):
    - **publish_value()** - Publishes a value to NetworkTables, given a key and a value. The type doesn't need additional methods; if a type is not supported, type checking will notify you.
    - **tunable()** - Defines and automatically updates a value that can be tuned through NetworkTables and reflects the value in the codebase, given a directory and a default value.
    - **tunablePID()** - Defines and automatically updates a PIDController object through NetworkTables and reflects the object in the codebase, given a directory and default PID values.
- **Full SysID support.** - instead of relying on the SysIdRoutine from commands2, we have one that works just as well for *any* architecture.
## Summary:

This is a great pick for teams that would like more control over their codebase, while prioritizing safety to consistently ensure reliability during competitions.

The best way to understand the structure and benefits of it is by looking at examples. I highly encourage you to look at this three-subsystem bot to understand the full architecture. The codebase can be found here: https://github.com/igowuu/5113-Perry

Thank you for taking your time to read this. I'd love to hear any feedback on this, so contact me if you'd like. To reach out, contact me through my discord: `"igowu."`
