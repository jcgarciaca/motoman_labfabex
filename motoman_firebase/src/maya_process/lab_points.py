
from geometry_msgs.msg import Point

home = Point(
    x=0,
    y=.8,
    z=1.9
)


def get_buffer_approach(buffer_idx):
    # buffer = _buffers[buffer_idx]
    # buffer.z += .15
    return Point(
        x=1,
        y=1,
        z=1.5
    )


buffer_1 = Point(
    x=1,
    y=0,
    z=1
)
buffer_2 = Point(
    x=1,
    y=0,
    z=1
)
buffer_3 = Point(
    x=1,
    y=0,
    z=1
)

_buffers = [buffer_1, buffer_2, buffer_3]

agv_approach = Point(
    x=1,
    y=1,
    z=1.5
)
agv = Point()


# TODO: add more machines
