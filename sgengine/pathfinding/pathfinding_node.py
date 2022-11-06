from ..abstract_node import AbstractNode


class PathfindingNode(AbstractNode):
    """Node for handling pathfinding"""

    def __init__(self, name: str, *args, **kwargs) -> None:
        super().__init__(name, *args, **kwargs)
        self._world: Any = None
        self._path: List[Any] = []
        self._path_publisher: Optional[Publisher] = None
        self._world_subscriber: Optional[Subscription] = None

    def _main(self):
        self._path_publisher = self.publish("path", [])
        self._world_subscriber = self.subscribe("world", self._world_callback, World)
