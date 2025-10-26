package lib.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lib.math.geometry.FieldConstants;

/** Wrapper over two generic HID boards for the button panel layout. */
public final class CommandButtonBoard {
  private final CommandGenericHID leftButtonBoard;
  private final CommandGenericHID rightButtonBoard;

  public CommandButtonBoard(int leftBoardPort, int rightBoardPort) {
    this.leftButtonBoard = new CommandGenericHID(leftBoardPort);
    this.rightButtonBoard = new CommandGenericHID(rightBoardPort);
  }

  public Trigger getReefButton(FieldConstants.Reef.ReefFace face, FieldConstants.Reef.Side side) {
    return switch (face) {
      case S1 -> switch (side) {
        case LEFT -> leftButtonBoard.button(10);
        case CENTER -> leftButtonBoard.button(11);
        case RIGHT -> leftButtonBoard.button(12);
      };
      case S2 -> switch (side) {
        case LEFT -> rightButtonBoard.button(7);
        case CENTER -> rightButtonBoard.button(8);
        case RIGHT -> rightButtonBoard.button(9);
      };
      case S3 -> switch (side) {
        case LEFT -> rightButtonBoard.button(11);
        case CENTER -> rightButtonBoard.button(12);
        case RIGHT -> rightButtonBoard.button(10);
      };
      case S4 -> switch (side) {
        case LEFT -> leftButtonBoard.button(1);
        case CENTER -> leftButtonBoard.button(2);
        case RIGHT -> leftButtonBoard.button(3);
      };
      case S5 -> switch (side) {
        case LEFT -> leftButtonBoard.button(4);
        case CENTER -> leftButtonBoard.button(5);
        case RIGHT -> leftButtonBoard.button(6);
      };
      case S6 -> switch (side) {
        case LEFT -> leftButtonBoard.button(7);
        case CENTER -> leftButtonBoard.button(8);
        case RIGHT -> leftButtonBoard.button(9);
      };
    };
  }

  public Trigger getActionButton() {
    return rightButtonBoard.button(1);
  }

  public Trigger getStowButton() {
    return rightButtonBoard.button(6);
  }

  public Trigger getL1Button() {
    return rightButtonBoard.button(5);
  }

  public Trigger getL2Button() {
    return rightButtonBoard.button(4);
  }

  public Trigger getL3Button() {
    return rightButtonBoard.button(3);
  }

  public Trigger getL4Button() {
    return rightButtonBoard.button(2);
  }
}
