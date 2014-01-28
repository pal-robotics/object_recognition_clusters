"""
Module defining several outputs for the object recognition pipeline
"""

from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward
from object_recognition_core.io.sink import SinkBase
from object_recognition_ros import init_ros
from object_recognition_ros.ecto_cells.ecto_object_recognition_msgs import Publisher_RecognizedObjectArray
from object_recognition_clusters.ecto_cells.io_clusters import PointCloudMsgAssembler
import ecto

########################################################################################################################

class ObjectClustersPublisher(ecto.BlackBox, SinkBase):
    """
    Class publishing the clusters of tabletop on a RecognizedObjectsArray topic
    """
    def __init__(self, *args, **kwargs):
        init_ros()
        ecto.BlackBox.__init__(self, *args, **kwargs)
        SinkBase.__init__(self)

    @classmethod
    def declare_cells(cls, p):
        return {'msg_assembler': CellInfo(PointCloudMsgAssembler),
                'passthrough': ecto.PassthroughN(items=dict(pose_results='The final results'))
                }

    @staticmethod
    def declare_direct_params(p):
        p.declare('latched', 'Determines if the topics will be latched.', False)
        p.declare('recognized_object_array_topic', 'The ROS topic to use for the recognized object', 'recognized_object_array')

    @staticmethod
    def declare_forwards(_p):
        p = {}
        i = {'msg_assembler': [Forward('clusters3d'), Forward('image_message')],
             'passthrough': [Forward('pose_results')]}
        o = {}
        return (p,i,o)

    def configure(self, p, _i, _o):
        self._recognized_object_array = Publisher_RecognizedObjectArray(topic_name=p.recognized_object_array_topic, latched=p.latched)

    def connections(self, _p):
        # connect to a publishing cell
        connections = [ self.msg_assembler['msg'] >> self._recognized_object_array['input']]

        return connections
