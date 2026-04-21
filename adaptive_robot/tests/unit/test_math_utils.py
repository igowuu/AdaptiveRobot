import math
from adaptive_robot.utils.math_utils import (
    clamp,
    radians_to_degrees,
    degrees_to_radians,
    rotations_to_meters,
    meters_to_rotations,
    ticks_to_meters,
    meters_to_ticks,
    mps_to_rps,
    rps_to_mps,
    ticks_per_100ms_to_mps,
    mps_to_ticks_per_100ms,
    rps_to_radps,
    radps_to_rps,
    rotations_to_radians,
    radians_to_rotations,
    degrees_to_rotations,
    rotations_to_degrees,
    normalized_rotations_to_radians,
    radians_to_normalized_rotations,
    rps_to_rpm,
    rpm_to_rps,
)


class TestClamp:
    """
    Tests for the clamp function.
    """
    def test_clamp_value_in_range(self) -> None:
        """
        Test that values within range are unchanged.
        """
        assert clamp(5.0, 0.0, 10.0) == 5.0
    
    def test_clamp_value_below_lower(self) -> None:
        """
        Test that values below lower bound are clamped.
        """
        assert clamp(-5.0, 0.0, 10.0) == 0.0
    
    def test_clamp_value_above_upper(self) -> None:
        """
        Test that values above upper bound are clamped.
        """
        assert clamp(15.0, 0.0, 10.0) == 10.0
    
    def test_clamp_equal_bounds(self) -> None:
        """
        Test clamping with equal bounds.
        """
        assert clamp(5.0, 3.0, 3.0) == 3.0
    
    def test_clamp_negative_range(self) -> None:
        """
        Test clamping with negative bounds.
        """
        assert clamp(-5.0, -10.0, -1.0) == -5.0
        assert clamp(-15.0, -10.0, -1.0) == -10.0
        assert clamp(5.0, -10.0, -1.0) == -1.0


class TestAngleConversion:
    """
    Tests for angle conversion functions.
    """
    def test_radians_to_degrees_zero(self) -> None:
        """
        Test conversion of zero radians.
        """
        assert radians_to_degrees(0) == 0.0
    
    def test_radians_to_degrees_pi(self) -> None:
        """
        Test conversion of pi radians to 180 degrees.
        """
        assert abs(radians_to_degrees(math.pi) - 180.0) < 1e-10
    
    def test_radians_to_degrees_half_pi(self) -> None:
        """
        Test conversion of pi/2 radians.
        """
        assert abs(radians_to_degrees(math.pi / 2) - 90.0) < 1e-10
    
    def test_degrees_to_radians_zero(self) -> None:
        """
        Test conversion of zero degrees.
        """
        assert degrees_to_radians(0) == 0.0
    
    def test_degrees_to_radians_180(self) -> None:
        """
        Test conversion of 180 degrees to pi radians.
        """
        assert abs(degrees_to_radians(180) - math.pi) < 1e-10
    
    def test_degrees_to_radians_90(self) -> None:
        """
        Test conversion of 90 degrees to pi/2 radians.
        """
        assert abs(degrees_to_radians(90) - math.pi / 2) < 1e-10
    
    def test_angle_conversion_roundtrip(self) -> None:
        """
        Test that conversions are reversible.
        """
        original = 45.0
        converted = radians_to_degrees(degrees_to_radians(original))
        assert abs(converted - original) < 1e-10


class TestRotationConversion:
    """
    Tests for rotation to distance conversions.
    """
    def test_rotations_to_meters_basic(self) -> None:
        """
        Test basic rotation to meters conversion.
        """
        result = rotations_to_meters(1.0, 1.0)
        assert abs(result - math.pi) < 1e-10
    
    def test_rotations_to_meters_with_gear_ratio(self) -> None:
        """
        Test rotation conversion with gear ratio.
        """
        result = rotations_to_meters(10.0, 1.0, 2.0)
        assert abs(result - 5 * math.pi) < 1e-10
    
    def test_rotations_to_meters_zero(self) -> None:
        """
        Test zero rotations.
        """
        result = rotations_to_meters(0.0, 1.0)
        assert result == 0.0
    
    def test_meters_to_rotations_basic(self) -> None:
        """
        Test basic meters to rotations conversion.
        """
        result = meters_to_rotations(math.pi, 1.0)
        assert abs(result - 1.0) < 1e-10
    
    def test_meters_to_rotations_with_gear_ratio(self) -> None:
        """
        Test meters to rotations with gear ratio.
        """
        result = meters_to_rotations(math.pi, 1.0, 2.0)
        assert abs(result - 2.0) < 1e-10
    
    def test_rotation_conversion_roundtrip(self) -> None:
        """
        Test that conversions are reversible.
        """
        original_rotations = 5.0
        wheel_diameter = 0.1
        converted = meters_to_rotations(
            rotations_to_meters(original_rotations, wheel_diameter),
            wheel_diameter
        )
        assert abs(converted - original_rotations) < 1e-10


class TestTickConversion:
    """
    Tests for encoder tick conversions.
    """
    def test_ticks_to_meters_basic(self) -> None:
        """
        Test basic tick to meters conversion.
        """
        # 1024 ticks per rotation, 1 meter diameter wheel
        # 1000 ticks = (1000/1024) rotations = (1000/1024) * pi * 1 meters
        result = ticks_to_meters(1024.0, 1024.0, 1.0)
        assert abs(result - math.pi) < 1e-10
    
    def test_ticks_to_meters_with_gear_ratio(self) -> None:
        """
        Test tick conversion with gear ratio.
        """
        result = ticks_to_meters(1024.0, 1024.0, 1.0, 2.0)
        # 1024 ticks = 1 motor rotation = 0.5 wheel rotations
        assert abs(result - 0.5 * math.pi) < 1e-10
    
    def test_ticks_to_meters_zero(self) -> None:
        """
        Test zero ticks.
        """
        result = ticks_to_meters(0, 1024.0, 1.0)
        assert result == 0.0
    
    def test_meters_to_ticks_basic(self) -> None:
        """
        Test meters to ticks conversion.
        """
        result = meters_to_ticks(math.pi, 1024.0, 1.0)
        assert abs(result - 1024.0) < 1e-10
    
    def test_meters_to_ticks_with_gear_ratio(self) -> None:
        """
        Test meters to ticks with gear ratio.
        """
        result = meters_to_ticks(math.pi, 1024.0, 1.0, 2.0)
        # pi meters with 2:1 = 2 wheel rotations = 2048 motor ticks
        assert abs(result - 2048.0) < 1e-10


class TestVelocityConversion:
    """
    Tests for velocity conversions.
    """
    def test_mps_to_rps_basic(self) -> None:
        """
        Test meters per second to rotations per second.
        """
        result = mps_to_rps(1.0, 1.0)
        assert abs(result - 1.0 / math.pi) < 1e-10
    
    def test_mps_to_rps_with_gear_ratio(self) -> None:
        """
        Test m/s to rps conversion with gear ratio.
        """
        result = mps_to_rps(1.0, 1.0, 2.0)
        assert abs(result - 2.0 / math.pi) < 1e-10
    
    def test_rps_to_mps_basic(self) -> None:
        """
        Test rotations per second to meters per second.
        """
        result = rps_to_mps(1.0, 1.0)
        assert abs(result - math.pi) < 1e-10
    
    def test_rps_to_mps_with_gear_ratio(self) -> None:
        """
        Test rps to m/s with gear ratio.
        """
        result = rps_to_mps(2.0, 1.0, 2.0)
        assert abs(result - math.pi) < 1e-10
    
    def test_velocity_conversion_roundtrip(self) -> None:
        """
        Test that velocity conversions are reversible.
        """
        original_mps = 2.5
        wheel_diameter = 0.1
        converted = rps_to_mps(
            mps_to_rps(original_mps, wheel_diameter),
            wheel_diameter
        )
        assert abs(converted - original_mps) < 1e-10


class TestTicksPerUpdateConversion:
    """
    Tests for ticks per 100ms conversions.
    """
    def test_ticks_per_100ms_to_mps_basic(self) -> None:
        """
        Test ticks per 100ms to m/s conversion.
        """
        result = ticks_per_100ms_to_mps(1024.0, 1024.0, 1.0)
        assert abs(result - 10.0 * math.pi) < 1e-10
    
    def test_ticks_per_100ms_to_mps_with_gear_ratio(self) -> None:
        """
        Test ticks per 100ms conversion with gear ratio.
        """
        result = ticks_per_100ms_to_mps(1024.0, 1024.0, 1.0, 2.0)
        assert abs(result - 5.0 * math.pi) < 1e-10
    
    def test_ticks_per_100ms_to_mps_zero(self) -> None:
        """
        Test zero ticks per update.
        """
        result = ticks_per_100ms_to_mps(0, 1024.0, 1.0)
        assert result == 0.0
    
    def test_mps_to_ticks_per_100ms_basic(self) -> None:
        """
        Test m/s to ticks per 100ms conversion.
        """
        result = mps_to_ticks_per_100ms(10.0 * math.pi, 1024.0, 1.0)
        assert abs(result - 1024.0) < 1e-10


class TestRotationAngleConversion:
    """
    Tests for rotation to angle conversions.
    """
    def test_rps_to_radps_basic(self) -> None:
        """
        Test RPS to rad/s conversion.
        """
        result = rps_to_radps(1.0)
        assert abs(result - 2.0 * math.pi) < 1e-10
    
    def test_rps_to_radps_with_gear_ratio(self) -> None:
        """
        Test RPS to rad/s with gear ratio.
        """
        result = rps_to_radps(2.0, 2.0)
        assert abs(result - 2.0 * math.pi) < 1e-10
    
    def test_radps_to_rps_basic(self) -> None:
        """
        Test rad/s to RPS conversion.
        """
        result = radps_to_rps(2.0 * math.pi)
        assert abs(result - 1.0) < 1e-10
    
    def test_rotations_to_radians_basic(self) -> None:
        """
        Test rotations to radians conversion.
        """
        result = rotations_to_radians(1.0)
        assert abs(result - 2.0 * math.pi) < 1e-10
    
    def test_radians_to_rotations_basic(self) -> None:
        """
        Test radians to rotations conversion.
        """
        result = radians_to_rotations(2.0 * math.pi)
        assert abs(result - 1.0) < 1e-10


class TestDegreesRotationConversion:
    """
    Tests for degrees and rotations conversions.
    """
    def test_degrees_to_rotations_basic(self) -> None:
        """
        Test degrees to rotations conversion.
        """
        result = degrees_to_rotations(360.0)
        assert abs(result - 1.0) < 1e-10
    
    def test_degrees_to_rotations_partial(self) -> None:
        """
        Test partial degrees to rotations.
        """
        result = degrees_to_rotations(180.0)
        assert abs(result - 0.5) < 1e-10
    
    def test_rotations_to_degrees_basic(self) -> None:
        """
        Test rotations to degrees conversion.
        """
        result = rotations_to_degrees(1.0)
        assert abs(result - 360.0) < 1e-10
    
    def test_degrees_rotations_roundtrip(self) -> None:
        """
        Test degrees/rotations conversions are reversible.
        """
        original = 90.0
        converted = rotations_to_degrees(degrees_to_rotations(original))
        assert abs(converted - original) < 1e-10


class TestNormalizedRotationConversion:
    """
    Tests for normalized rotation conversions.
    """
    def test_normalized_rotations_to_radians_basic(self) -> None:
        """
        Test normalized rotation to radians.
        """
        result = normalized_rotations_to_radians(1.0)
        assert abs(result - 2.0 * math.pi) < 1e-10
    
    def test_normalized_rotations_to_radians_half(self) -> None:
        """
        Test half normalized rotation.
        """
        result = normalized_rotations_to_radians(0.5)
        assert abs(result - math.pi) < 1e-10
    
    def test_normalized_rotations_to_radians_negative(self) -> None:
        """
        Test negative normalized rotation.
        """
        result = normalized_rotations_to_radians(-1.0)
        assert abs(result + 2.0 * math.pi) < 1e-10
    
    def test_radians_to_normalized_rotations_basic(self) -> None:
        """
        Test radians to normalized rotation.
        """
        result = radians_to_normalized_rotations(2.0 * math.pi)
        assert abs(result - 1.0) < 1e-10
    
    def test_normalized_rotation_roundtrip(self) -> None:
        """
        Test normalized rotation conversions are reversible.
        """
        original = 0.75
        converted = radians_to_normalized_rotations(
            normalized_rotations_to_radians(original)
        )
        assert abs(converted - original) < 1e-10


class TestRPMConversion:
    """
    Tests for RPM conversions.
    """
    def test_rps_to_rpm_basic(self) -> None:
        """
        Test RPS to RPM conversion.
        """
        result = rps_to_rpm(1.0)
        assert abs(result - 60.0) < 1e-10
    
    def test_rpm_to_rps_basic(self) -> None:
        """
        Test RPM to RPS conversion.
        """
        result = rpm_to_rps(60.0)
        assert abs(result - 1.0) < 1e-10
    
    def test_rpm_rps_roundtrip(self) -> None:
        """
        Test RPM/RPS conversions are reversible.
        """
        original_rps = 50.0
        converted = rpm_to_rps(rps_to_rpm(original_rps))
        assert abs(converted - original_rps) < 1e-10

