import SG_Primitives as P
from SymbolicEntity import SymbolicEntity
from SymbolicProperty import SymbolicProperty
from functools import partial

EGO = partial(P.filterByAttr, "G", "name", "ego")
HUMAN = SymbolicEntity('person_1', ['person'])

def person_in_direction(robot, person, direction_label):
    """
    Checks if the person is located in a specific direction relative to the robot.
    """
    direction_set = partial(P.relSet, robot, direction_label)
    overlapping_entities = partial(P.intersection, direction_set, person)
    return partial(P.gt, partial(P.size, overlapping_entities), 0)


passing_human_on_left = SymbolicProperty(
    "robot_must_pass_human_on_the_left",
    "((is_front & !is_behind) -> (!is_left U is_behind))",
    [
        ("is_front", person_in_direction(EGO, HUMAN, "FRONT")),
        ("is_left",  person_in_direction(EGO, HUMAN, "LEFT")),
        ("is_behind", person_in_direction(EGO, HUMAN, "BACK"))
    ],
    [HUMAN]
)
all_human_properties = [passing_human_on_left]
