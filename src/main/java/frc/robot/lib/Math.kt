package frc.robot.lib

import edu.wpi.first.math.Vector
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.units.measure.Distance
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sin

inline val Double.signSquare
    get() = this * abs(this)

operator fun Vector<N2>.component1(): Double {
    return this[0]
}

operator fun Vector<N2>.component2(): Double {
    return this[1]
}

/**
 * Returns the pitch diameter of a #25 chain sprocket
 *
 * @param teeth The number of teeth on the sprocket
 */
fun pd25(teeth: Int): Distance {
    return (.25 / sin(PI / teeth)).inches
}
