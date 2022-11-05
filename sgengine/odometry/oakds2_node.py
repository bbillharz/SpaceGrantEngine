from ..abstract_node import AbstractNode


class OakDS2Node(AbstractNode):
    """Node for handling the OAKD-S2 camera"""

    def __init__(self, name: str, *args, **kwargs) -> None:
        super().__init__(name, *args, **kwargs)

        # TODO: define pipeline structures
        pass

    def _main(self):
        # TODO: define pubs and subs
        pass
