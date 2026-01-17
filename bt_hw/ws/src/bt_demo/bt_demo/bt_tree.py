from __future__ import annotations

from dataclasses import dataclass

import py_trees
from py_trees.common import Status, Access


@dataclass(frozen=True)
class Topics:
    estop: str = "/bt_demo/safety/estop"
    battery_level: str = "/bt_demo/battery/level"
    has_task: str = "/bt_demo/task/has_task"
    active_top: str = "/bt_demo/debug/active_top"


class BlackboardKeys:
    ESTOP = "safety_estop"
    BATTERY_LEVEL = "battery_level"
    HAS_TASK = "task_has_task"


def _writer_client(name: str) -> py_trees.blackboard.Client:
    c = py_trees.blackboard.Client(name=name)
    c.register_key(key=BlackboardKeys.ESTOP, access=Access.WRITE)
    c.register_key(key=BlackboardKeys.BATTERY_LEVEL, access=Access.WRITE)
    c.register_key(key=BlackboardKeys.HAS_TASK, access=Access.WRITE)
    return c


def _reader_client(name: str) -> py_trees.blackboard.Client:
    c = py_trees.blackboard.Client(name=name)
    c.register_key(key=BlackboardKeys.ESTOP, access=Access.READ)
    c.register_key(key=BlackboardKeys.BATTERY_LEVEL, access=Access.READ)
    c.register_key(key=BlackboardKeys.HAS_TASK, access=Access.READ)
    return c


class ConditionEStop(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "Cond: E-Stop"):
        super().__init__(name)
        self.bb = _reader_client(self.name)

    def update(self) -> Status:
        return Status.SUCCESS if bool(getattr(self.bb, BlackboardKeys.ESTOP, False)) else Status.FAILURE


class ConditionLowBattery(py_trees.behaviour.Behaviour):
    def __init__(self, threshold: float, name: str = "Cond: Low battery"):
        super().__init__(name)
        self.threshold = float(threshold)
        self.bb = _reader_client(self.name)

    def update(self) -> Status:
        level = float(getattr(self.bb, BlackboardKeys.BATTERY_LEVEL, 1.0))
        return Status.SUCCESS if level < self.threshold else Status.FAILURE


class ConditionHasTask(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "Cond: Task?"):
        super().__init__(name)
        self.bb = _reader_client(self.name)

    def update(self) -> Status:
        return Status.SUCCESS if bool(getattr(self.bb, BlackboardKeys.HAS_TASK, False)) else Status.FAILURE


class ActionAlwaysRunning(py_trees.behaviour.Behaviour):
    def update(self) -> Status:
        return Status.RUNNING


class ActionHalt(ActionAlwaysRunning):
    pass


class ActionDockAndCharge(ActionAlwaysRunning):
    pass


class ActionNavigate(ActionAlwaysRunning):
    pass


class ActionDetectItem(ActionAlwaysRunning):
    pass


class ActionGrasp(ActionAlwaysRunning):
    pass


class ActionPlaceToTote(ActionAlwaysRunning):
    pass


class ActionStowArms(ActionAlwaysRunning):
    pass


def build_pick_and_place_subtree() -> py_trees.behaviour.Behaviour:
    seq = py_trees.composites.Sequence(name="Seq: Pick&Place", memory=False)
    seq.add_children(
        [
            ActionDetectItem(name="Action: Detect item"),
            ActionGrasp(name="Action: Grasp"),
            ActionPlaceToTote(name="Action: Place to tote"),
            ActionStowArms(name="Action: Stow arms"),
        ]
    )
    return seq


def build_tree(low_battery_threshold: float = 0.2) -> py_trees.trees.BehaviourTree:
    w = _writer_client("bt_demo_init")
    setattr(w, BlackboardKeys.ESTOP, False)
    setattr(w, BlackboardKeys.BATTERY_LEVEL, 1.0)
    setattr(w, BlackboardKeys.HAS_TASK, False)

    root = py_trees.composites.Selector(name="Sel: Root", memory=False)

    seq_safe = py_trees.composites.Sequence(name="Seq: Safety", memory=False)
    seq_safe.add_children([ConditionEStop(), ActionHalt(name="Action: Halt")])

    seq_charge = py_trees.composites.Sequence(name="Seq: Charge", memory=False)
    seq_charge.add_children(
        [
            ConditionLowBattery(threshold=low_battery_threshold),
            ActionDockAndCharge(name="Action: Dock and charge"),
        ]
    )

    seq_order = py_trees.composites.Sequence(name="Seq: Order", memory=False)
    seq_order.add_children(
        [
            ConditionHasTask(),
            ActionNavigate(name="Action: Navigate"),
            build_pick_and_place_subtree(),
        ]
    )

    idle = ActionAlwaysRunning(name="Action: Idle")

    root.add_children([seq_safe, seq_charge, seq_order, idle])

    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=1.0)
    return tree


def get_active_top_branch_name(tree: py_trees.trees.BehaviourTree) -> str:
    root = tree.root
    for child in root.children:
        if child.status in (Status.RUNNING, Status.SUCCESS):
            return child.name
    return "none"

