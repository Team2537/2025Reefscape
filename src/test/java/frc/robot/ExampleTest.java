package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ExampleTest {
  @Test
  void stringLowerCaseShouldReturnAllLowerCase() {
    assertEquals("robot", "Robot".toLowerCase());
  }

  @Test
  void twoPlusTwoEqualsFour() {
    assertEquals(4, 2 + 2);
  }
}
