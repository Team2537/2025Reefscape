package lib.controllers

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.Trigger
import lib.math.geometry.FieldConstants


// the button board has two controllers on different ports, each is wired to 12 different buttons. This class is used to map the buttons to CommandTriggers
class CommandButtonBoard(
    private val leftBoardPort: Int,
    private val rightBoardPort: Int
) {
    private val leftButtonBoard = CommandGenericHID(leftBoardPort)
    private val rightButtonBoard = CommandGenericHID(rightBoardPort)
    
    // N, S, NE, NW, SE, and SW each have 3 buttons one for each side (center, right, left)
    // other buttons are ACTION, STOW, L1, L2, L3, L4

    // S1 = S
    // S2 = SE
    // S3 = NE
    // S4 = N
    // S5 = NW
    // S6 = SW

    fun getReefButton(face: FieldConstants.Reef.ReefFace, side: FieldConstants.Reef.Side): Trigger {
        return when (face) {
            FieldConstants.Reef.ReefFace.S1 -> when (side) { // South
                FieldConstants.Reef.Side.LEFT -> leftButtonBoard.button(6)
                FieldConstants.Reef.Side.CENTER -> leftButtonBoard.button(4)
                FieldConstants.Reef.Side.RIGHT -> leftButtonBoard.button(5)
            }
            FieldConstants.Reef.ReefFace.S2 -> when (side) { // Southeast
                FieldConstants.Reef.Side.LEFT -> rightButtonBoard.button(7)
                FieldConstants.Reef.Side.CENTER -> rightButtonBoard.button(8)
                FieldConstants.Reef.Side.RIGHT -> rightButtonBoard.button(9)
            }
            FieldConstants.Reef.ReefFace.S3 -> when (side) { // Northeast
                FieldConstants.Reef.Side.LEFT -> rightButtonBoard.button(11)
                FieldConstants.Reef.Side.CENTER -> rightButtonBoard.button(12)
                FieldConstants.Reef.Side.RIGHT -> rightButtonBoard.button(10)
            }
            FieldConstants.Reef.ReefFace.S4 -> when (side) { // North
                FieldConstants.Reef.Side.LEFT -> leftButtonBoard.button(3)
                FieldConstants.Reef.Side.CENTER -> leftButtonBoard.button(2)
                FieldConstants.Reef.Side.RIGHT -> leftButtonBoard.button(1)
            }
            FieldConstants.Reef.ReefFace.S5 -> when (side) { // Northwest
                FieldConstants.Reef.Side.LEFT -> leftButtonBoard.button(12)
                FieldConstants.Reef.Side.CENTER -> leftButtonBoard.button(11)
                FieldConstants.Reef.Side.RIGHT -> leftButtonBoard.button(10)
            }
            FieldConstants.Reef.ReefFace.S6 -> when (side) { // Southwest
                FieldConstants.Reef.Side.LEFT -> leftButtonBoard.button(9)
                FieldConstants.Reef.Side.CENTER -> leftButtonBoard.button(8)
                FieldConstants.Reef.Side.RIGHT -> leftButtonBoard.button(7)
            }
        }
    }

    fun getActionButton(): Trigger {
        return rightButtonBoard.button(1)
    }

    fun getStowButton(): Trigger {
        return rightButtonBoard.button(6)
    }

    fun getL1Button(): Trigger {
        return rightButtonBoard.button(5)
    }

    fun getL2Button(): Trigger {
        return rightButtonBoard.button(4)
    }

    fun getL3Button(): Trigger {
        return rightButtonBoard.button(3)
    }

    fun getL4Button(): Trigger {
        return rightButtonBoard.button(2)
    }

}