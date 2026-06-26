import SG_Primitives as P
from SymbolicEntity import SymbolicEntity
from SymbolicProperty import SymbolicProperty
from functools import partial

EGO = partial(P.filterByAttr, "G", "name", "ego")
CHAIR = SymbolicEntity('chair_1', ['chair'])

def person_in_direction(robot, chair, direction_label):
    """
    Checks if the person is located in a specific direction relative to the robot.
    """
    direction_set = partial(P.relSet, robot, direction_label)
    overlapping_entities = partial(P.intersection, direction_set, chair)
    return partial(P.gt, partial(P.size, overlapping_entities), 0)


passing_chair_on_right = SymbolicProperty(
    "robot_must_pass_chair_on_the_right",
    "((is_front & !is_behind) -> (!is_right U is_behind))",
    [
        ("is_front", person_in_direction(EGO, CHAIR, "FRONT")),
        ("is_right",  person_in_direction(EGO, CHAIR, "RIGHT")),
        ("is_behind", person_in_direction(EGO, CHAIR, "BACK"))
    ],
    [CHAIR]
)
all_chair_properties = [passing_chair_on_right]
