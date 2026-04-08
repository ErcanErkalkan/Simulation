from simulation_app.domain.vector import Vector


class Valuation:
    """
    Provides methods for calculating valuations based on time, distance, and a composite of both.
    """

    @staticmethod
    def time_based_valuation(
        t: float, t_threshold1: float, t_threshold2: float
    ) -> float:
        """
        Computes a time-based valuation.

        :param t: Current time or elapsed time.
        :param t_threshold1: First time threshold, after which valuation starts increasing.
        :param t_threshold2: Second time threshold, after which valuation grows quadratically.
        :return: A valuation based on time.
        """
        if t < 0:
            raise ValueError("Time (t) cannot be negative.")
        if t_threshold1 > t_threshold2:
            raise ValueError("t_threshold1 must be less than or equal to t_threshold2.")

        if t <= t_threshold1:
            return 0
        elif t_threshold1 < t <= t_threshold2:
            return t - t_threshold1
        else:
            return (t - t_threshold2) ** 2 + (t_threshold2 - t_threshold1)

    @staticmethod
    def distance_based_valuation(
        u_pos: Vector, g_pos: Vector, epsilon: float = 0.0001
    ) -> float:
        """
        Computes a distance-based valuation.

        :param u_pos: Position of the UAV as a Vector.
        :param g_pos: Position of the goal as a Vector.
        :param epsilon: Small value to avoid division by zero.
        :return: A valuation inversely proportional to the distance between u_pos and g_pos.
        """
        if not isinstance(u_pos, Vector) or not isinstance(g_pos, Vector):
            raise TypeError("u_pos and g_pos must be instances of Vector.")

        distance = ((u_pos.x - g_pos.x) ** 2 + (u_pos.y - g_pos.y) ** 2) ** 0.5
        return 1 / (distance + epsilon)

    @staticmethod
    def composite_valuation(
        t: float,
        u_pos: Vector,
        g_pos: Vector,
        t_threshold1: float,
        t_threshold2: float,
        epsilon: float = 0.0001,
        time_weight: float = 1.0,
        distance_weight: float = 1.0,
    ) -> float:
        """
        Computes a composite valuation based on time and distance.

        :param t: Current time or elapsed time.
        :param u_pos: Position of the UAV as a Vector.
        :param g_pos: Position of the goal as a Vector.
        :param t_threshold1: First time threshold, after which valuation starts increasing.
        :param t_threshold2: Second time threshold, after which valuation grows quadratically.
        :param epsilon: Small value to avoid division by zero in distance calculation.
        :param time_weight: Scaling factor for time-based valuation.
        :param distance_weight: Scaling factor for distance-based valuation.
        :return: A composite valuation combining time and distance metrics.
        """
        time_value = (
            Valuation.time_based_valuation(t, t_threshold1, t_threshold2) * time_weight
        )
        distance_value = (
            Valuation.distance_based_valuation(u_pos, g_pos, epsilon) * distance_weight
        )
        return time_value * distance_value
