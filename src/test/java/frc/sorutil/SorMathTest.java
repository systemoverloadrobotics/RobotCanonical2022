package frc.sorutil;

import static org.junit.Assert.*;
import org.junit.Test;

public class SorMathTest {
  @Test
  public void polarTest() {

  }

  @Test
  public void linearInterpolateTest() {
    double result = SorMath.linearInterpolate(20, 10, 30, 100, 300);

    assertTrue("Result from interpolate is approximately equal to expected",
        SorMath.epsilonEquals(result, 200));

    result = SorMath.linearInterpolate(-10, 10, 30, 100, 300);
    assertTrue(
        "Result from interpolate is approximately equal to expected when extrapolated negative",
        SorMath.epsilonEquals(result, -100));

    boolean caught = false;
    try {
      result = SorMath.linearInterpolate(-10, 10, 10, 100, 300);
    } catch (IllegalArgumentException e) {
      caught = true;
    }
    if (!caught) {
      fail("linearInterpolate should throw an exception when the scale arguments are zero!");
    }

    result = SorMath.linearInterpolate(20, 10, 30, 100, 100);
    assertTrue(
        "Result from interpolate is approximately equal to expected when low == high",
        SorMath.epsilonEquals(result, 100));

  }
}
