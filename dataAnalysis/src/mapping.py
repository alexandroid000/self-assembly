
class Mapping():
    def __init__(self, X_MAX, Y_MAX, Z_MAX):
        self.X_MAX = X_MAX
        self.Y_MAX = Y_MAX
        self.Z_MAX = Z_MAX

    def map3Dto1D(self,x,y,yaw):
        return int((yaw*self.X_MAX*self.Y_MAX) + (y*self.X_MAX) + x)

    def map1Dto3D(self,element):
        z = int(element / (self.X_MAX * self.Y_MAX));
        element -= int(z * self.X_MAX * self.Y_MAX)
        y = int(element / self.X_MAX)
        x = int(element % self.X_MAX)
        return ( x, y, z );

    def checkValid1DMap(self, element):
        if(element > (self.X_MAX*self.Y_MAX*self.Z_MAX-1) or element < 0):
            return 0
        else:
            return 1
        
    def checkValid3DMap(self,x,y,yaw):
        if(x < 0 or y < 0 or yaw < 0 or x >= self.X_MAX or y >= self.Y_MAX or yaw >= self.Z_MAX):
            return 0
        return self.checkValid1DMap(self.map3Dto1D(x,y,yaw))
if __name__ == "__main__":
    mapping = Mapping(5,4,3)
    if(mapping.map1Dto3D(mapping.map3Dto1D(1,1,1) == (1,1,1))):
        print("Mapping works!")
