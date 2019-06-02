import math


class Delta:
    def __init__(self, upperLinkLength, lowerLinkLength, baseRadius, footRadius, groundToFootLength, offsetX=0, offsetY=0, offsetZ=0):
        self.l1 = upperLinkLength
        self.l2 = lowerLinkLength
        self.plusNX = -groundToFootLength
        self.plusNY = footRadius - baseRadius
        self.ndLimit = self.l1 + self.l2
        self.offsetX = offsetX
        self.offsetY = offsetY
        self.offsetZ = offsetZ

    def pointToAngle(self, pointX, pointY, pointZ):
        x = pointX + self.offsetX
        y = pointY + self.offsetY
        z = pointZ + self.offsetZ
        
        # motor angle 1

        rx = x
        ry = y
        nx = -z + self.plusNX
        ny = rx + self.plusNY
        nl2 = math.sqrt(self.l2 * self.l2 - ry * ry)
        nd = math.sqrt(nx * nx + ny * ny)
        #if nd > self.ndLimit:
        #    return None
        motorAngle1 = math.acos((self.l1 * self.l1 + nd * nd - nl2 * nl2) / (2 * self.l1 * nd)) + math.atan2(ny, nx)

        # motor angle 2
        # rx and ry is rotated x and y. + pi/3
        rx = ((-x + (math.sqrt(3) * y)) / 2)
        ry = ((-y - (math.sqrt(3) * x)) / 2)
        #nx = -z + self.plusNX
        ny = rx + self.plusNY
        nl2 = math.sqrt(self.l2 * self.l2 - ry * ry)
        nd = math.sqrt(nx * nx + ny * ny)
        #if nd > self.ndLimit:
        #    return None
        motorAngle2 = math.acos((self.l1 * self.l1 + nd * nd - nl2 * nl2) / (2 * self.l1 * nd)) + math.atan2(ny, nx)

        # motor angle 3
        # rx and ry is rotated x and y. + 2*pi/3
        rx = ((-x - (math.sqrt(3) * y)) / 2)
        ry = ((-y + (math.sqrt(3) * x)) / 2)
        # nx = -z + self.plusNX
        ny = rx + self.plusNY
        nl2 = math.sqrt(self.l2 * self.l2 - ry * ry)
        nd = math.sqrt(nx * nx + ny * ny)
        #if nd > self.ndLimit:
        #    return None
        motorAngle3 = math.acos((self.l1 * self.l1 + nd * nd - nl2 * nl2) / (2 * self.l1 * nd)) + math.atan2(ny, nx)

        return (motorAngle1, motorAngle2, motorAngle3)
