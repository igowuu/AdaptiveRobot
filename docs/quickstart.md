# Quickstart guide

## 1- Lifecycle of a Robot
`AdaptiveRobot` is based on WPILib's `TimedRobot` and so you still have a  `Robot` class, this time inheriting from `AdaptiveRobot`, and that class then overrides some framework provided lifecycle hooks.
```python
class Robot(AdaptiveRobot):
  def onTeleopPeriodic():
    # this function is called once every cycle durring teleop
    pass
  def onAutonomousInit():
    # this function is called ONCE when auto starts
    pass
```
There are way more of these lifecycle hooks (all avalible [here](docs/features/ROBOT_GUIDE.md)), but the astute amongst you will notice they all follow this basic pattern, to get the lifecycle hook name just do "on" + STATE_NAME + WHEN, so if I want the lifecylce hook to run whenver I exit disabled mode it is `onDisabledExit` or if I want to run code when the tests start it is `onTestInit`. This is all you realy need for a simple bot, but when you start getting more complicated programs that need to be split into multiple files, you can do that with Components.


## 2- Components
A component is fundementaly just a unit of state and logic that powers a specific part of the robot. A component can be as broad as a `Drivetrain` coomponent that orchestrates wheels and odometry or as small as a `SwerveWheel` that handles low level motor stuff. As a general rule you should have a robot class, which then controls "logic components" ie components that calcuate what should be done, which then in turn `Request` a "state component" to control the physical motors to do this. This creates a nice hirerarchy and one way dataflow which is easy to reason about at scale. (If you are currioius the CS term for this is a DAG or Directed Acyclcic Graph). Now lets look at a sample `AdaptiveRobot` Component and discect its parts

```python
from adaptive_robot import AdaptiveComponent
from phoenix6.hardware import TalonFX
class Intake(AdaptiveComponent):
    motor: TalonFX
    speed: float # Note: Unlike MagicBot these feilds are not strictly nesscary (there is no injection)
    def __init__(self) -> None:
        super().__init__()
        self.motor = TalonFX(0)  # CAN ID 0
        self.speed = 0.0

    def set_speed(self, speed: float) -> None:
        self.speed = speed

    def execute(self) -> None:
        self.motor.set(self.speed)

    def on_disabled(self) -> None:
        self.speed = 0.0
```

Now notice how the `Intake` class inherits from `AdaptiveComponent`, this is so that the component is auto registered with the Robot, and so telemetry and error handling works properly (discussed in later sections). `AdaptiveComponents` are fairly straight foward, you can (and should) use whatever component strucutre or division of responsibility works best for your team. It is totaly valid to not have a `set_speed` method and just have a `turn_on` method instead, where the intake then calcuates its own appropriate speed. 

Once you have your component written, calling it from the robot is easy!
```python
from wpilib import XboxControler
# from <YOUR_COMPONENT_PATH> import Intake
class MyRobot(AdaptiveRobot):
    controler: XboxControler
    intake: Intake
    def onRobotInit(self) -> None:
        self.intake = Intake()
        self.controler = XboxControler(0) # USB Port 0
    def onTeleopPeriodic(self) -> None:
        if self.controller.getAButton():
            self.intake.set_speed(0.8)
        else:
            self.intake.set_speed(0.0)
        //self.intake.execute() will be automaticaly called by the scheduler
```
UI/UX designers have long had a principle called "gradual disclosre of complexity", meaning that if the user does not dig that deep, they should be be greated with an avalanche of complexity. `AdaptiveComponent` is a great example of this, if you just need a simple component like that, it just works and is dead simple. However as we will see in the next few sections, it can also add features to become incredibly powerfull.

## 3- Component Communication
Note: The name `AxisControler` will be changed to `RequestArbiter` or `RequestControler` before the 1.0 release. It was orginaly made for joysticks only, the name does not make sense now that it's use has been expanded.

Lets imagine you have two "parent" components that you want to control a single "child" component (although this is usualy bad design), or that you have an error system and a main component that run asynchronosly. The naieve thing to do would be to simply both have them issue thier commands, and whatever arives first/last wins. This is quite literaly the texbook definition of a race condition and it can cause massive instability and occasionaly failures. `AdaptiveRobot` eliminates this catagory of issue, with a request arbiting condition called `AxisControler`.

Imagine a safety monitor and a main controller both need to control the same intake motor. 
Without Arbitration:
```python
# Both of these run every iteration, last one wins unpredictably
class IntakeSafety(AdaptiveComponent):
    def execute(self):
        if self.current_sensor.get() > 40:
            self.intake.motor.set(0.0)  # jam detected, cut power

class IntakeController(AdaptiveComponent):
    def execute(self):
        self.intake.motor.set(self.requested_speed)  # might overwrite safety!
```

With Arbitration
```python
class IntakeSafety(AdaptiveComponent):
    def execute(self):
        if self.current_sensor.get() > 40:
            self.intake.request_speed(0.0, BasicPriority.SAFETY, "jam_detector") #the final string is just for debuging so you know where the request comes from. 

class IntakeController(AdaptiveComponent):
    def execute(self):
        self.intake.request_speed(self.speed, BasicPriority.TELEOP, "main_controller")

class Intake(AdaptiveComponent):
    def __init__(self):
        super().__init__()
        self.motor = TalonFX(0)
        self.speed = AxisController()

    def request_speed(self, speed, priority, source):
        self.speed.request(speed, priority, source)

    def execute(self):
        self.motor.set(self.speed.resolve().value)  # safety always wins, registration order irrelevant
```
That `BasicPriority` enum you saw contains
```python
class BasicPriority(Enum):
    SAFETY = 3    # highest
    AUTO = 2
    TELEOP = 1    # lowest
```
You can also overwrite it with your own, or use any `IntEnum` you like!

# 4- Errors, Error Handling and `FaultManager`
Sometimes, something goes wrong. A motor overheats, a cable comes lose, a sense is not tuned properly. The traditonal way to handle this is using pythongs `raise`, `try`, and `except` keywords. This approach works, but lets be honest, you will not rember to catch every fault, and when you do it will probobly not be handled properly. `AdaptiveRobot` introduces a system called `Faults` to manage this. Anything that inherits from the `Faultable` class can raise a fault (any `AdaptiveRobot` or `AdaptiveComponent` already inherit from faultable. Every fault has to have a sevarity level, either `Warning` (operation continues, error logged), `Error` (an error that can be handled), and `Critical` (robot immedatly disables). Lets look at an example
```python
from adaptive_robot import AdaptiveComponent, Faultable, FaultSeverity

class MyComponent(AdaptiveComponent):
    def execute(self) -> None:
        try:
            result = self.sensor.read()
        except SensorError as e:
            self.raise_fault( # Faultable provides the raise_fault method
                component=self,
                severity=FaultSeverity.ERROR,
                description="Sensor read failed",
                exception=e
            )

    def check_limits(self) -> None:
        if self.position > MAX_POSITION:
            self.raise_fault(
                component=self,
                severity=FaultSeverity.WARNING,
                description=f"Position limit exceeded: {self.position}"
            )

    def detect_critical_issue(self) -> None:
        self.raise_fault(
            component=self,
            severity=FaultSeverity.CRITICAL,
            description="Motor overheating detected"
        )
```
Faults are handled by the `FaultManager` class, which sits above the robot in the component heirerarchy. `FaultManager` is provided by `AdaptiveRobot` and end users should rarley touch it, but if you look in the full fault docs, you can see how to do it. When you raise a fault it is caught by `FaultManager`, if it is `Critical`, the fault robot is disabled. If it is an `Error` the component is marked as unhealty and `on_faulted_periodic` is called every cycle. After the max unealthy cyclces (configurable, 10 by default) the component is disabled. Warnings are logged and control returns to the component. Errors can and should be explicitly caught by components if possible, that said you can also handle errors in `on_faulted_periodic`. 
