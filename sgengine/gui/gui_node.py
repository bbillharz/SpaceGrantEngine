from ..abstract_node import AbstractNode

from std_msgs import String


class GUINode(AbstractNode):
    """The node for updating the GUI."""

    def __init__(self, name: str, *args, **kwargs) -> None:
        super().__init__(name, *args, **kwargs)

        # TODO: define web server stuff and whatevers
        pass

    def _main(self):
        # TODO: handle opening a web server and publishing stuff to it
        self.publish("testing", self._testing_generator, String, 10)

        self.subscribe("testing", self._testing_callback, String)
        pass

    def _testing_generator(self):
        while True:
            yield String(data="testing")

    def _testing_callback(self, msg: String):
        print(msg.data)
