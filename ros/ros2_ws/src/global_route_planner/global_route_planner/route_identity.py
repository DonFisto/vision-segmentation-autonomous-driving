"""ROS-independent mission and route-revision identity.

A mission starts only after a goal has been accepted.
Replanning creates route revisions without changing mission identity.

Mission IDs combine a per-process random nonce with a goal counter.
They are probabilistically distinct across planner restarts, not
permanent globally guaranteed identifiers.
"""

from dataclasses import dataclass
import secrets

UINT32_MAX = (1 << 32) - 1
UINT64_MAX = (1 << 64) - 1


def _check_uint(value, name, maximum, minimum=0):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{name} outside valid unsigned integer range")


@dataclass(frozen=True)
class RouteIdentity:
    mission_id: int
    route_id: int
    map_revision: int

    def __post_init__(self):
        _check_uint(self.mission_id, "mission_id", UINT64_MAX, 1)
        _check_uint(self.route_id, "route_id", UINT64_MAX, 1)
        _check_uint(self.map_revision, "map_revision", UINT64_MAX)


class RouteIdentitySequence:
    """Assign identifiers to accepted goals and published routes."""

    def __init__(self, map_revision: int, session_nonce: int | None = None):
        _check_uint(map_revision, "map_revision", UINT64_MAX)

        if session_nonce is None:
            session_nonce = 0
            while session_nonce == 0:
                session_nonce = secrets.randbits(32)

        _check_uint(session_nonce, "session_nonce", UINT32_MAX, 1)

        self._map_revision = map_revision
        self._session_nonce = session_nonce
        self._mission_counter = 0
        self._route_counter = 0

    @property
    def current_mission_id(self) -> int | None:
        if self._mission_counter == 0:
            return None

        return (
            (self._session_nonce << 32)
            | self._mission_counter
        )

    def accept_goal(self) -> int:
        """Call only after a goal passes association validation."""
        if self._mission_counter >= UINT32_MAX:
            raise OverflowError("Mission counter exhausted")

        self._mission_counter += 1
        return self.current_mission_id

    def issue_route(self) -> RouteIdentity:
        """Create a revision, successful or unsuccessful."""
        if self.current_mission_id is None:
            raise RuntimeError("Cannot issue route without accepted goal")

        if self._route_counter >= UINT64_MAX:
            raise OverflowError("Route counter exhausted")

        self._route_counter += 1

        return RouteIdentity(
            mission_id=self.current_mission_id,
            route_id=self._route_counter,
            map_revision=self._map_revision,
        )
