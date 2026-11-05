package frc.robot.lib

import edu.wpi.first.math.Vector
import edu.wpi.first.math.numbers.N2
import kotlin.math.abs

inline val Double.signSquare
    get() = this * abs(this)

operator fun Vector<N2>.component1(): Double {
    return this[0]
}

operator fun Vector<N2>.component2(): Double {
    return this[1]
}
