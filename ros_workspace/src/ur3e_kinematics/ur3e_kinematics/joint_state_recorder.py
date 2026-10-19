"""Nodo que escucha /joint_states y se queda con las articulaciones del brazo."""
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_STATES_TOPIC: str = "/joint_states"
QUEUE_DEPTH: int = 10
NANOSEC_TO_SEC: float = 1e-9
LOG_PERIOD_SEC: float = 1.0
ARM_JOINTS: List[str] = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]


class JointStateRecorder(Node):
    """Suscriptor que extrae las articulaciones del brazo de /joint_states."""

    def __init__(self) -> None:
        super().__init__("joint_state_recorder")
        self._subscription = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._joint_state_callback, QUEUE_DEPTH
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        """Procesa un mensaje; ignora los que no son del brazo (p. ej. la pinza)."""
        index_by_name: Dict[str, int] = {name: i for i, name in enumerate(msg.name)}
        if not all(joint in index_by_name for joint in ARM_JOINTS):
            return
        positions = [msg.position[index_by_name[j]] for j in ARM_JOINTS]
        has_velocity = len(msg.velocity) == len(msg.name)
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * NANOSEC_TO_SEC
        self.get_logger().info(
            f"t={timestamp:.3f} velocity_available={has_velocity} pos={positions}",
            throttle_duration_sec=LOG_PERIOD_SEC,
        )


def main(args: Optional[List[str]] = None) -> None:
    """Punto de entrada del nodo."""
    rclpy.init(args=args)
    node = JointStateRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()