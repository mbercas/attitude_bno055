import Wireframe_EKF as wf
import pygame
from operator import itemgetter
import readSensor_EKF as rs

class ProjectionViewer:
    """ Displays 3D objects on a Pygame screen """
    def __init__(self, width, height, wireframe):
        self.width = width
        self.height = height
        self.wireframe = wireframe
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption('Attitude Determination using Quaternions')
        self.background = (10,10,50)
        self.clock = pygame.time.Clock()
        pygame.font.init()
        self.font = pygame.font.SysFont('Comic Sans MS', 30)

    def run(self, sensorInstance):
        """ Create a pygame screen until it is closed. """
        running = True
        loopRate = 50
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    sensorInstance.close()
            self.clock.tick(loopRate)
            data, cal  = sensorInstance.getSerialData()
            self.cal = cal
            # Don't update the wireframe until the IMU starts calibrating
            if (cal[0] + cal[1] + cal[2] + cal[3]) > 0:
                self.wireframe.quatRotate([data[0], data[1], data[2]],
                                          [data[3], data[4], data[5]],
                                          [data[6], data[7], data[8]],
                                          1/loopRate)
            self.display()
            pygame.display.flip()

    def display(self):
        """ Draw the wireframes on the screen. """
        self.screen.fill(self.background)

        # Calibration
        self.messageDisplay(f"CS: {self.cal[0]}",
                            self.screen.get_width()*0.05,
                            self.screen.get_height()*0,
                            (65, 105, 225))      # Crimson
        self.messageDisplay(f"CG: {self.cal[1]}",
                            self.screen.get_width()*0.05,
                            self.screen.get_height()*0.05,
                            (65, 105, 225))      # Crimson
        self.messageDisplay(f"CA: {self.cal[2]}",
                            self.screen.get_width()*0.05,
                            self.screen.get_height()*0.1,
                            (65, 105, 225))      # Crimson
        self.messageDisplay(f"CM: {self.cal[3]}",
                            self.screen.get_width()*0.05,
                            self.screen.get_height()*0.15,
                            (65, 105, 225))      # Crimson
        
        # Get the current attitude
        yaw, pitch, roll = self.wireframe.getAttitude()
        self.messageDisplay("Yaw: %.1f" % yaw,
                            self.screen.get_width()*0.75,
                            self.screen.get_height()*0,
                            (220, 20, 60))      # Crimson
        self.messageDisplay("Pitch: %.1f" % pitch,
                            self.screen.get_width()*0.75,
                            self.screen.get_height()*0.05,
                            (0, 255, 255))     # Cyan
        self.messageDisplay("Roll: %.1f" % roll,
                            self.screen.get_width()*0.75,
                            self.screen.get_height()*0.1,
                            (65, 105, 225))    # Royal Blue

        # Transform nodes to perspective view
        dist = 5
        pvNodes = []
        pvDepth = []
        for node in self.wireframe.nodes:
            point = [node.x, node.y, node.z]
            newCoord = self.wireframe.rotatePoint(point)
            comFrameCoord = self.wireframe.convertToComputerFrame(newCoord)
            pvNodes.append(self.projectOthorgraphic(comFrameCoord[0], comFrameCoord[1], comFrameCoord[2],
                                                    self.screen.get_width(), self.screen.get_height(),
                                                    70, pvDepth))
            """
            pvNodes.append(self.projectOnePointPerspective(comFrameCoord[0], comFrameCoord[1], comFrameCoord[2],
                                                           self.screen.get_width(), self.screen.get_height(),
                                                           5, 10, 30, pvDepth))
            """

        # Calculate the average Z values of each face.
        avg_z = []
        for face in self.wireframe.faces:
            n = pvDepth
            z = (n[face.nodeIndexes[0]] + n[face.nodeIndexes[1]] +
                 n[face.nodeIndexes[2]] + n[face.nodeIndexes[3]]) / 4.0
            avg_z.append(z)
        # Draw the faces using the Painter's algorithm:
        for idx, val in sorted(enumerate(avg_z), key=itemgetter(1)):
            face = self.wireframe.faces[idx]
            pointList = [pvNodes[face.nodeIndexes[0]],
                         pvNodes[face.nodeIndexes[1]],
                         pvNodes[face.nodeIndexes[2]],
                         pvNodes[face.nodeIndexes[3]]]
            pygame.draw.polygon(self.screen, face.color, pointList)

    # One vanishing point perspective view algorithm
    def projectOnePointPerspective(self, x, y, z, win_width, win_height, P, S, scaling_constant, pvDepth):
        # In Pygame, the y axis is downward pointing.
        # In order to make y point upwards, a rotation around x axis by 180 degrees is needed.
        # This will result in y' = -y and z' = -z
        xPrime = x
        yPrime = -y
        zPrime = -z
        xProjected = xPrime * (S/(zPrime+P)) * scaling_constant + win_width / 2
        yProjected = yPrime * (S/(zPrime+P)) * scaling_constant + win_height / 2
        pvDepth.append(1/(zPrime+P))
        return (round(xProjected), round(yProjected))

    # Normal Projection
    def projectOthorgraphic(self, x, y, z, win_width, win_height, scaling_constant, pvDepth):
        # In Pygame, the y axis is downward pointing.
        # In order to make y point upwards, a rotation around x axis by 180 degrees is needed.
        # This will result in y' = -y and z' = -z
        xPrime = x
        yPrime = -y
        xProjected = xPrime * scaling_constant + win_width / 2
        yProjected = yPrime * scaling_constant + win_height / 2
        # Note that there is no negative sign here because our rotation to computer frame
        # assumes that the computer frame is x-right, y-up, z-out
        # so this z-coordinate below is already in the outward direction
        pvDepth.append(z)
        return (round(xProjected), round(yProjected))

    def messageDisplay(self, text, x, y, color):
        textSurface = self.font.render(text, True, color, self.background)
        textRect = textSurface.get_rect()
        textRect.topleft = (x, y)
        self.screen.blit(textSurface, textRect)

def initializeCube():
    block = wf.Wireframe()

    block_nodes = [(x, y, z) for x in (-1.5, 1.5) for y in (-1, 1) for z in (-0.1, 0.1)]
    node_colors = [(255, 255, 255)] * len(block_nodes)
    block.addNodes(block_nodes, node_colors)
    block.outputNodes()

    faces = [(0, 2, 6, 4), (0, 1, 3, 2), (1, 3, 7, 5), (4, 5, 7, 6), (2, 3, 7, 6), (0, 1, 5, 4)]
    colors = [(255, 0, 255), (255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 255, 0)]
    block.addFaces(faces, colors)
    block.outputFaces()

    return block


if __name__ == '__main__':
    portName = '/dev/ttyUSB0'
    # portName = 'COM6'
    baudRate = 115200
    dataNumBytes = 4*3*3 + 4  # number of bytes of 1 data point
    numParams = 9 # number of plots in 1 graph
    s = rs.SerialRead(portName, baudRate, dataNumBytes, numParams)  # initializes all required variables
    s.readSerialStart()  # starts background thread

    block = initializeCube()
    pv = ProjectionViewer(640, 480, block)
    pv.run(s)
