import pybullet as p
import math

DEFAULT_POS = [0., 0., 0.]
DEFAULT_ORI = [0., 0., 0., 1.]


def tunnel(length, width, wall_width=0.05, position=DEFAULT_POS, orientation=DEFAULT_ORI):
    # create a rectangular tunnel as a building block of an obstacle parcour

    half_extents = [[length / 2, width / 2 + wall_width, wall_width / 2],
                    [length / 2, width / 2 + wall_width, wall_width / 2],
                    [length / 2, wall_width / 2, width / 2],
                    [length / 2, wall_width / 2, width / 2]]
    positions = [[0, 0., -width / 2 - wall_width / 2],
                 [0, 0., width / 2 + wall_width / 2],
                 [0, -width / 2 - wall_width / 2, 0.],
                [0, width / 2 + wall_width / 2, 0.]]

    visual_shape_id = p.createVisualShapeArray(shapeTypes=[p.GEOM_BOX] * 4,
                                               halfExtents=half_extents,
                                               visualFramePositions=positions
                                               )
    collision_shape_id = p.createCollisionShapeArray(shapeTypes=[p.GEOM_BOX] * 4,
                                                     halfExtents=half_extents,
                                                     collisionFramePositions=positions
                                                     )

    mb = p.createMultiBody(baseMass=0.,
                           baseInertialFramePosition=[0, 0, 0],
                           baseCollisionShapeIndex=collision_shape_id,
                           baseVisualShapeIndex=visual_shape_id,
                           basePosition=position,
                           baseOrientation=orientation,
                           useMaximalCoordinates=False)
    p.changeVisualShape(mb, -1, rgbaColor=[1, 1, 1, 0.5], specularColor=[0.4, 0.4, 0])
    return mb


def corner(width, wall_width=0.05, position=DEFAULT_POS, orientation=DEFAULT_ORI):
    # create a corner (cube open at neg. x and pos. z) as a building block of an obstacle parcour
    half_extents = [[width / 2, width / 2 + wall_width, wall_width / 2],
                    [width / 2, wall_width / 2, width / 2],
                    [width / 2, wall_width / 2, width / 2],
                    [wall_width / 2, width / 2 + wall_width, width / 2]]
    positions = [[0, 0., -width / 2 - wall_width / 2],
                 [0, -width / 2 - wall_width / 2, 0.],
                 [0, width / 2 + wall_width / 2, 0.],
                 [width/2 + wall_width / 2, 0., 0.]]

    visual_shape_id = p.createVisualShapeArray(shapeTypes=[p.GEOM_BOX] * 4,
                                               halfExtents=half_extents,
                                               visualFramePositions=positions
                                               )
    collision_shape_id = p.createCollisionShapeArray(shapeTypes=[p.GEOM_BOX] * 4,
                                                     halfExtents=half_extents,
                                                     collisionFramePositions=positions
                                                     )

    mb = p.createMultiBody(baseMass=0.,
                           baseInertialFramePosition=[0, 0, 0],
                           baseCollisionShapeIndex=collision_shape_id,
                           baseVisualShapeIndex=visual_shape_id,
                           basePosition=position,
                           baseOrientation=orientation,
                           useMaximalCoordinates=False)
    p.changeVisualShape(mb, -1, rgbaColor=[1, 1, 1, 0.5], specularColor=[0.4, 0.4, 0])
    return mb


def cap(width, wall_width=0.05, position=DEFAULT_POS, orientation=DEFAULT_ORI):
    # create a cap that can be used to close corners or tunnels.
    # By default it is a wall in the y-z-plane that can be rotated and translated to the desired position
    # create a corner (cube open at neg. x and pos. z) as a building block of an obstacle parcour
    half_extents = [wall_width / 2, width / 2 + wall_width, width / 2]
    pos = [0, 0., 0.]
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                          halfExtents=half_extents,
                                          visualFramePosition=pos
                                          )
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                halfExtents=half_extents,
                                                collisionFramePosition=pos
                                                )

    mb = p.createMultiBody(baseMass=0.,
                           baseInertialFramePosition=[0, 0, 0],
                           baseCollisionShapeIndex=collision_shape_id,
                           baseVisualShapeIndex=visual_shape_id,
                           basePosition=position,
                           baseOrientation=orientation,
                           useMaximalCoordinates=False)
    p.changeVisualShape(mb, -1, rgbaColor=[1, 1, 1, 0.5], specularColor=[0.4, 0.4, 0])
    return mb


def trunk(x, y, width, height, orientation=DEFAULT_ORI):
    half_extents = [width, width, height]
    pos = [0, 0., 0.]
    position = [x, y, height]
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                          halfExtents=half_extents,
                                          visualFramePosition=pos
                                          )
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                halfExtents=half_extents,
                                                collisionFramePosition=pos
                                                )

    mb = p.createMultiBody(baseMass=0.,
                           baseInertialFramePosition=[0, 0, 0],
                           baseCollisionShapeIndex=collision_shape_id,
                           baseVisualShapeIndex=visual_shape_id,
                           basePosition=position,
                           baseOrientation=orientation,
                           useMaximalCoordinates=False)
    p.changeVisualShape(mb, -1, rgbaColor=[1, 1, 1, 0.5], specularColor=[0.4, 0.4, 0])
    return mb


def leaves(x, y, width, height, orientation=DEFAULT_ORI):
    half_extents = [width * 1.4, width * 1.4, width * 0.8]
    pos = [0, 0., 0.]
    position = [x, y, height * 2 + width * 0.8]
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                          halfExtents=half_extents,
                                          visualFramePosition=pos
                                          )
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                halfExtents=half_extents,
                                                collisionFramePosition=pos
                                                )

    mb = p.createMultiBody(baseMass=0.,
                           baseInertialFramePosition=[0, 0, 0],
                           baseCollisionShapeIndex=collision_shape_id,
                           baseVisualShapeIndex=visual_shape_id,
                           basePosition=position,
                           baseOrientation=orientation,
                           useMaximalCoordinates=False)
    p.changeVisualShape(mb, -1, rgbaColor=[1, 1, 1, 0.5], specularColor=[0.4, 0.4, 0])
    return mb


def tree(x=0., y=0., width=1., height=1., orientation=DEFAULT_ORI):
    return leaves(x, y, width, height, orientation), trunk(x, y, width, height, orientation)


def rotate_z(angle):
    return p.getQuaternionFromAxisAngle(axis=[0, 0, 1], angle=angle)


def trees(base_x=-3.5, base_y=-4.2):
    tree(x=base_x + 1, y=base_y + 1, width=0.5, height=1, orientation=rotate_z(0.3))
    tree(x=base_x - 1, y=base_y + 1, width=0.5, height=1.3)
    tree(x=base_x, y= base_y + 3, width=0.4, height=0.9, orientation=rotate_z(0.4))
    tree(x=base_x - 1.4, y=base_y + 4, width=0.6, height=1)
    tree(x=base_x + 1.5, y=base_y + 4.1, width=0.5, height=1, orientation=rotate_z(0.1))


def tunnels():
    # create an obstacle parcour

    # width of the tunnels
    width = 1

    # create some orientation quaternions to rotate tunnels and corners
    tunnel_ornx = [0., 0., 0., 1.]
    tunnel_orny = p.getQuaternionFromAxisAngle(axis=[0, 0, 1], angle=math.pi / 2)
    tunnel_ornz = p.getQuaternionFromAxisAngle(axis=[0, 1, 0], angle=math.pi / 2)
    corner_negxposz = [0., 0., 0., 1.]
    corner_negzposy = p.getQuaternionFromEuler([math.pi, 0.0, -math.pi / 2])
    corner_negyposx = p.getQuaternionFromEuler([math.pi / 2, 0.0, math.pi / 2])

    # create the parcour
    cap(width=width, position=[-1., 0., 2.])
    tunnel(length=5., width=width, position=[1.5, 0., 2.], orientation=tunnel_ornx)
    corner(width=width, position=[4.5, 0., 2.], orientation=corner_negxposz)
    tunnel(length=2., width=width, position=[4.5, 0.0, 3.5], orientation=tunnel_ornz)
    corner(width=width, position=[4.5, 0., 5.0], orientation=corner_negzposy)
    tunnel(length=4., width=width, position=[4.5, 2.5, 5], orientation=tunnel_orny)
    corner(width=width, position=[4.5, 5., 5.], orientation=corner_negyposx)
    tunnel(length=4., width=width, position=[7.0, 5.0, 5.], orientation=tunnel_ornx)
    cap(width=width, position=[9., 5., 5.])


def wall(x=0., y=0., z=0., width=1., height=1., thickness=0.01, orientation=DEFAULT_ORI):
    half_extents = [width / 2, thickness, height / 2]
    position = [x, y, z + height / 2]
    pos = [0, 0., 0.]
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                                          halfExtents=half_extents,
                                          visualFramePosition=pos
                                          )
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                halfExtents=half_extents,
                                                collisionFramePosition=pos
                                                )

    mb = p.createMultiBody(baseMass=0.,
                           baseInertialFramePosition=[0, 0, 0],
                           baseCollisionShapeIndex=collision_shape_id,
                           baseVisualShapeIndex=visual_shape_id,
                           basePosition=position,
                           baseOrientation=orientation,
                           useMaximalCoordinates=False)
    p.changeVisualShape(mb, -1, rgbaColor=[1, 1, 1, 0.5], specularColor=[0.4, 0.4, 0])
    return mb


def window(base_x=0., base_y=0., base_z=0., width=3., height=3.):
    wall(x=base_x + width / 3, y=base_y, z=base_z + height / 3, width=width / 3, height=2 * height / 3)
    wall(x=base_x + width / 6, y=base_y, z=base_z, width=2 * width / 3, height=height / 3)
    wall(x=base_x - width / 6, y=base_y, z=base_z + 2 * height / 3, width=2 * width / 3, height=height / 3)
    wall(x=base_x - width / 3, y=base_y, z=base_z, width=width / 3, height=height * 2 / 3)


def three_wall(base_x=0., base_y=0., width=1., height=1.):
    wall(x=base_x - 2 * width, y=base_y, width=2 * width, height=height)
    wall(x=base_x, y=base_y, width=width, height=height)
    wall(x=base_x + 2 * width, y=base_y, width=2 * width, height=height)


def box(size=8):
    orientation_y = p.getQuaternionFromAxisAngle(axis=[0, 1, 0], angle=math.pi / 2)
    cap(width=size, position=[0, 0, size], orientation=orientation_y)
    tunnel(length=size, width=size, position=[0, 0, size / 2], orientation=orientation_y)

def obstacle():
    # three_wall(base_y=1.0)
    # window(base_y=1.0, width=6.)
    # trees()
    # tunnels()
    box()
