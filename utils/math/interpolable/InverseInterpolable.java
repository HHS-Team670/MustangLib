package frc.team670.mustanglib.utils.math.interpolable;

/**
 * @see <a href=
 * "https://github.com/Team254/FRC-2017-Public/blob/master/src/com/team254/lib/util/InverseInterpolable.java">
 * Taken from 254's 2017 code.</a>
 * 
 * InverseInterpolable is an interface used by an Interpolating Tree as the Key
 * type. Given two endpoint keys and a third query key, an InverseInterpolable
 * object can calculate the interpolation parameter of the query key on the
 * interval [0, 1].
 * 
 * @param <T> The Type of InverseInterpolable
 * @see InterpolatingTreeMap
 */
public interface InverseInterpolable<T> {
    /**
     * Given this point (lower), a query point (query), and an upper point (upper),
     * estimate how far (on [0, 1]) between 'lower' and 'upper' the query point
     * lies.
     * 
     * @param upper
     * @param query
     * @return The interpolation parameter on [0, 1] representing how far between
     *         this point and the upper point the query point lies.
     */
    public double inverseInterpolate(T upper, T query);
}